"""PX4 local-NED Offboard flight; no ROS, serial I/O, or GPS dependency here.

This is a reference controller, not flight-qualified software. PX4 remains
responsible for arming checks, landing detection, and failsafe decisions.
"""

import math
from collections import deque
from dataclasses import dataclass, field

MAX_SPEED = 0.1  # Position-reference speed, NOT a guarantee of actual speed.
YAW_RATE = math.radians(15)
YAW_TOL = math.radians(5)
POSITION_TOL = 0.05
SETTLED_SPEED = 0.05


@dataclass
class Telemetry:
    mode: str = "UNKNOWN"
    armed: bool = False
    landed: str = "UNKNOWN"
    position: tuple | None = None
    velocity: tuple | None = None
    yaw: float | None = None
    estimate_ok: bool = False
    estimator_flags: int | None = None
    heartbeat_at: float = -math.inf
    landed_at: float = -math.inf
    position_at: float = -math.inf
    attitude_at: float = -math.inf
    estimator_at: float = -math.inf
    parameters: dict = field(default_factory=dict)  # name -> (float value, receive time)

    def navigation_ok(self, now):
        return (now - self.heartbeat_at < 3 and now - self.landed_at < 1
                and now - self.position_at < 0.5 and now - self.attitude_at < 0.5
                and now - self.estimator_at < 2 and self.estimate_ok
                and self.position is not None and self.velocity is not None
                and self.yaw is not None
                and all(math.isfinite(v) for v in (*self.position, *self.velocity, self.yaw)))

    def telemetry_age_s(self, now):
        ages = {}
        for name, stamp in (("HEARTBEAT", self.heartbeat_at),
                            ("LOCAL_POSITION_NED", self.position_at),
                            ("ATTITUDE", self.attitude_at),
                            ("EXTENDED_SYS_STATE", self.landed_at),
                            ("ESTIMATOR_STATUS", self.estimator_at)):
            age = now - stamp
            ages[name] = round(age, 3) if math.isfinite(age) else None
        return ages

    def navigation_issues(self, now):
        """Explain the controller's gate; this is not Commander readiness."""
        issues = []
        for name, stamp, limit in (("HEARTBEAT", self.heartbeat_at, 3.),
                                   ("LOCAL_POSITION_NED", self.position_at, .5),
                                   ("ATTITUDE", self.attitude_at, .5),
                                   ("EXTENDED_SYS_STATE", self.landed_at, 1.),
                                   ("ESTIMATOR_STATUS", self.estimator_at, 2.)):
            age = now - stamp
            if not math.isfinite(age):
                issues.append(f"未收到 {name}")
            elif age >= limit:
                issues.append(f"{name} 過期：{age:.3f}s（需 < {limit:g}s）")
        for name, value in (("位置", self.position), ("速度", self.velocity)):
            if value is None or not all(math.isfinite(v) for v in value):
                issues.append(f"{name}數值無效")
        if self.yaw is None or not math.isfinite(self.yaw):
            issues.append("yaw 數值無效")
        if not self.estimate_ok:
            flags = self.estimator_flags
            if flags is None:
                issues.append("尚無 ESTIMATOR_STATUS flags 可判讀")
            else:
                details = []
                # MAVLink ESTIMATOR_STATUS_FLAGS; matches the existing adapter
                # mask exactly. Neither absolute horizontal position nor GPS
                # is mandatory: the relative horizontal position bit suffices.
                for bit, name in ((1, "姿態"), (2, "水平速度"), (4, "垂直速度"), (32, "垂直位置")):
                    if not flags & bit:
                        details.append(name + "未標為有效")
                if not flags & (8 | 16):
                    details.append("水平位置未標為有效（相對/絕對皆無）")
                if flags & 128:
                    details.append("CONST_POS_MODE=1")
                if flags & 2048:
                    details.append("ACCEL_ERROR=1")
                issues.append(f"估計器檢查未通過 flags={flags} (0x{flags:04x})："
                              + ("、".join(details) or "請核對接收處的有效性判斷"))
        return issues


class Line:
    """Half-cosine time profile along a straight segment in local NED."""

    def __init__(self, start_ned, offset_neu):
        self.start = start_ned
        x, y, up = offset_neu
        self.delta = (x, y, -up)
        self.end = tuple(a + b for a, b in zip(self.start, self.delta))
        self.duration = math.pi * math.hypot(*self.delta) / (2 * MAX_SPEED)
        if not math.isfinite(self.duration) or any(abs(x) > 3.4e38 for x in self.end):
            raise ValueError("位移超出可表示範圍")

    def at(self, elapsed):
        if elapsed >= self.duration:
            return self.end
        fraction = math.sin(math.pi * max(0., elapsed) / (2 * self.duration)) ** 2
        return tuple(a + fraction * b for a, b in zip(self.start, self.delta))


class Flight:
    def __init__(self, telemetry, send_command, send_target, report,
                 takeoff_height=0.3, land_speed=0.1, landing_timeout=120.):
        for value in (takeoff_height, land_speed, landing_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("起飛高度、降落速度、逾時必須為正有限數值")
        if takeoff_height < 0.2 or land_speed > 0.1:
            raise ValueError("此範例起飛高度至少 0.2 m、降落指令速度最多 0.1 m/s")
        self.t, self.send_command, self.send_target, self.report = telemetry, send_command, send_target, report
        self.takeoff_height, self.land_speed, self.landing_timeout = takeoff_height, land_speed, landing_timeout
        self.phase = "IDLE"
        self.changed_at = 0.
        self.target = self.yaw_ref = self.line = self.last_send = None
        self.velocity_ref = None
        self.pending = None
        self.seen = deque(maxlen=256)
        self.settled_from = self.tracking_error_since = None
        self.ground_z = None
        self.start_yaw = 0.
        self.air_seen = False
        self.land_started = self.command_at = -math.inf
        self.kill_latched = False
        self.kill_requested_at = self.kill_last_sent = -math.inf
        self.kill_disarmed_at = None

    def enter(self, phase, now, note):
        self.phase, self.changed_at = phase, now
        self.settled_from = None
        self.pending = None
        self.report(note)

    def clear_targets(self):
        self.target = self.yaw_ref = self.line = self.last_send = self.velocity_ref = None
        self.tracking_error_since = None

    def fault(self, now, reason):
        if self.kill_latched:
            self.report("kill 鎖存中：" + reason)
            return
        self.clear_targets()
        self.enter("FAULT", now, reason + "；停止串流，由 PX4 執行設定的保護動作，不自動重啟")

    def issue(self, phase, command, now):
        self.enter(phase, now, f"已請求 {command}，等待遙測確認")
        self.pending, self.command_at = command, now
        self.send_command(command)

    def acknowledge(self, command, accepted, result, now):
        if command != self.pending:
            return
        # Command 400 ACKs cannot distinguish arm/normal disarm/force disarm.
        # Delayed arm ACKs must never confirm or cancel a later stop request.
        if command in {"disarm", "kill"}:
            self.report(f"ARM/DISARM ACK：{result}；仍等待後續 armed 狀態")
        elif accepted:
            self.report(f"{command}：{result}；仍等待遙測確認")
        else:
            self.fault(now, f"PX4 拒絕 {command}：{result}")

    def landing_setup_ok(self, now):
        values = {}
        for key in ("LNDMC_Z_VEL_MAX", "MPC_Z_VEL_MAX_DN"):
            value, stamp = self.t.parameters.get(key, (math.nan, -math.inf))
            if now - stamp >= 15 or not math.isfinite(value):
                raise ValueError(f"尚未讀到新鮮的飛控參數 {key}")
            values[key] = value
        # Reserve 20% of speed for lateral drift correction; keep vector norm
        # <= land_speed AND steady descending vz above PX4's detection threshold.
        min_descent = self.land_speed * math.sqrt(1 - 0.2 ** 2)
        if values["LNDMC_Z_VEL_MAX"] <= 0 or min_descent < 1.1 * values["LNDMC_Z_VEL_MAX"]:
            raise ValueError("降落速度與 LNDMC_Z_VEL_MAX 不相容；先驗證參數，不會自動修改飛控")
        if values["MPC_Z_VEL_MAX_DN"] < self.land_speed:
            raise ValueError("MPC_Z_VEL_MAX_DN 小於降落速度")

    def request(self, request, now):
        if not isinstance(request, dict):
            raise ValueError("請求必須是 JSON 物件")
        rid = request.get("id")
        if not isinstance(rid, str) or not 0 < len(rid) <= 128:
            raise ValueError("缺少有效指令 id")
        if rid in self.seen:
            return
        self.seen.append(rid)
        mode = request.get("mode")
        if mode == "kill":
            self.kill_latched = True
            self.kill_requested_at = self.kill_last_sent = now
            self.kill_disarmed_at = None
            self.clear_targets()
            self.issue("KILL_REQUEST", "kill", now)
            return
        if self.kill_latched:
            raise ValueError("kill 已鎖存；確認安全後重啟 controller")
        if now - self.t.heartbeat_at >= 3:
            raise ValueError("PX4 heartbeat 尚未就緒")
        if mode == "land":
            self.request_land(now)
            return
        if not self.t.navigation_ok(now):
            raise ValueError("遙測檢查未通過：" + "；".join(self.t.navigation_issues(now)))
        t = self.t
        if mode == "arm":
            if self.phase not in {"IDLE", "FAULT"} or t.armed or t.landed != "GROUND":
                raise ValueError("0 僅在地面未解鎖、無進行中操作時接受")
            self.landing_setup_ok(now)
            self.clear_targets()
            self.target, self.yaw_ref = t.position, t.yaw
            self.ground_z, self.air_seen = None, False
            self.enter("PRIME", now, "預送零速度目標；尚未解鎖、尚未起飛")
            return
        if mode == "takeoff":
            if self.phase != "ARMED_GROUND" or not t.armed or t.mode != "OFFBOARD" or t.landed != "GROUND":
                raise ValueError("先按 0，等待 ARMED_GROUND；若已自動上鎖需重新按 0")
            self.landing_setup_ok(now)
            height = request.get("height", self.takeoff_height)
            if type(height) not in (int, float) or not math.isfinite(height) or height < 0.2:
                raise ValueError("起飛高度必須是至少 0.2 m 的有限數值")
            line = Line(t.position, (0., 0., height))
            self.target, self.yaw_ref = t.position, t.yaw
            self.ground_z = t.position[2]
            self.line = line
            self.air_seen = False
            self.enter("TAKING_OFF", now, f"Offboard 起飛：相對本次起點上升 {height:.2f} m")
            return
        if mode != "offboard":
            raise ValueError("不明模式")
        values = [request.get(k) for k in "xyz"]
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError("x/y/z 必須是有限的公尺數")
        if self.phase != "HOLD" or t.mode != "OFFBOARD" or not t.armed or t.landed != "AIR":
            raise ValueError("2 僅在起飛完成、Offboard HOLD 等待指令時接受")
        if math.dist(t.position, self.target) > POSITION_TOL or math.hypot(*t.velocity) > SETTLED_SPEED:
            raise ValueError("尚未穩定停在上一個目標")
        line = Line(self.target, values)
        if self.ground_z is None or self.ground_z - line.end[2] < 0.15:
            raise ValueError("追蹤目標過低；請用 3 執行降落")
        self.line, self.start_yaw = line, self.yaw_ref
        self.enter("ALIGN", now, "對齊 heading 0°；對齊後直接移動")

    def request_land(self, now):
        t = self.t
        if self.phase in {"LAND_BRAKE", "DESCEND", "DISARMING", "GROUND_STOP"}:
            return
        # Cancel the arm sequence as well. Repeated NORMAL disarms cover an
        # in-flight arm command without confusing its ACK with stop completion.
        if self.phase in {"PRIME", "SELECT_OFFBOARD", "ARMING", "ARMED_GROUND"}:
            if now - t.landed_at >= 1 or t.landed != "GROUND":
                raise ValueError("無法確認在地面；不執行一般上鎖")
            self.issue("GROUND_STOP", "disarm", now)
            return
        if not t.armed:
            raise ValueError("飛機已上鎖，沒有降落動作")
        if not t.navigation_ok(now) or t.mode != "OFFBOARD":
            raise ValueError("Offboard 降落需要有效本地定位及目前為 OFFBOARD；查看 PX4 保護狀態")
        self.landing_setup_ok(now)
        self.target, self.yaw_ref = t.position, t.yaw
        self.line = None
        self.air_seen = self.air_seen or t.landed == "AIR"
        self.land_started = now
        self.tracking_error_since = None
        self.enter("LAND_BRAKE", now, "取消移動，先穩定位置，再用 Offboard 下降")

    def is_settled(self, condition, stamp, seconds):
        if not condition:
            self.settled_from = None
        elif self.settled_from is None:
            self.settled_from = stamp
        return self.settled_from is not None and stamp - self.settled_from >= seconds

    def emit(self, now, velocity=None):
        self.velocity_ref = velocity
        self.send_target(None if velocity is not None else self.target, self.yaw_ref, velocity)
        self.last_send = now

    def landing_velocity(self, now):
        t = self.t
        xy = [0.5 * (a - b) for a, b in zip(self.target[:2], t.position[:2])]
        length = math.hypot(*xy)
        cap = 0.2 * self.land_speed
        if length > cap:
            xy = [v * cap / length for v in xy]
        # A brief ramp is allowed; sustained descent then satisfies the detector.
        down = min(self.land_speed * min(1., (now - self.descent_started) / 1.0),
                   math.sqrt(max(0., self.land_speed ** 2 - math.hypot(*xy) ** 2)))
        return (*xy, down)

    def tick_kill(self, now):
        if self.phase != "KILL_REQUEST":
            return
        t = self.t
        fresh = now - t.heartbeat_at < 3 and t.heartbeat_at > self.kill_requested_at
        if fresh and not t.armed:
            if self.kill_disarmed_at is None:
                self.kill_disarmed_at = t.heartbeat_at
            elif t.heartbeat_at - self.kill_disarmed_at >= 1:
                self.enter("KILLED", now, "PX4 已回報上鎖；kill 鎖存，確認安全後重啟 controller")
                return
        else:
            self.kill_disarmed_at = None
        if now - self.kill_requested_at >= 3:
            self.enter("KILL_UNCONFIRMED", now, "kill 未確認；不代表已停轉，可再次輸入 kill")
        elif now - self.kill_last_sent >= .5:
            self.send_command("kill")
            self.kill_last_sent = now

    def tick(self, now):
        if self.kill_latched:
            self.tick_kill(now)
            return
        t, phase = self.t, self.phase
        if phase in {"IDLE", "FAULT"}:
            return
        if now - t.heartbeat_at >= 3:
            self.fault(now, "heartbeat 逾時")
            return
        elapsed = now - self.changed_at
        if phase == "GROUND_STOP":
            ground = now - t.landed_at < 1 and t.landed == "GROUND"
            if not ground:
                self.fault(now, "取消解鎖時地面狀態失效")
            elif self.is_settled(not t.armed and t.heartbeat_at > self.changed_at,
                                 t.heartbeat_at, 1.) and elapsed >= 2:
                self.clear_targets()
                self.enter("IDLE", now, "已取消操作，PX4 已上鎖")
            elif elapsed > 6:
                self.fault(now, "未確認地面上鎖")
            else:
                if now - self.command_at >= .5:
                    self.send_command("disarm")
                    self.command_at = now
                if t.navigation_ok(now) and t.mode == "OFFBOARD":
                    self.emit(now, (0., 0., 0.))
            return
        if phase in {"LAND_BRAKE", "DESCEND", "DISARMING"} and not t.armed:
            if t.heartbeat_at > self.land_started and t.landed_at > self.land_started and t.landed == "GROUND":
                self.clear_targets()
                self.enter("IDLE", now, "降落完成，PX4 已回報上鎖")
            else:
                self.fault(now, "降落期間上鎖，但尚未確認落地")
            return
        if phase == "ARMED_GROUND" and not t.armed:
            self.clear_targets()
            self.enter("IDLE", now, "PX4 已上鎖；若要起飛請重新按 0")
            return
        if not t.navigation_ok(now):
            self.fault(now, "遙測檢查未通過：" + "；".join(t.navigation_issues(now)))
            return
        if self.last_send is not None and now - self.last_send > .4:
            self.fault(now, "setpoint 串流間隔超過 0.4 s")
            return
        if phase in {"PRIME", "SELECT_OFFBOARD", "ARMING", "ARMED_GROUND"}:
            if t.landed != "GROUND":
                self.fault(now, "地面解鎖流程中偵測到離地")
                return
            if phase in {"PRIME", "SELECT_OFFBOARD"} and t.armed:
                self.fault(now, "解鎖流程被其他控制來源改變")
                return
            if phase == "PRIME" and elapsed >= 1.2 and self.last_send is not None:
                self.issue("SELECT_OFFBOARD", "offboard", now)
            elif phase == "SELECT_OFFBOARD":
                if t.mode == "OFFBOARD" and t.heartbeat_at > self.changed_at:
                    self.issue("ARMING", "arm", now)
                elif elapsed > 6:
                    self.fault(now, "未確認 Offboard；沒有送出 arm")
                    return
            elif phase in {"ARMING", "ARMED_GROUND"}:
                if t.mode != "OFFBOARD":
                    self.fault(now, "解鎖期間離開 Offboard")
                    return
                if phase == "ARMING" and t.armed and t.heartbeat_at > self.changed_at:
                    self.enter("ARMED_GROUND", now, "已解鎖，等待 1 起飛；馬達可能怠速旋轉")
                elif phase == "ARMING" and elapsed > 6:
                    self.fault(now, "未確認解鎖")
                    return
            # Zero vertical velocity, with z position ignored, cannot request
            # takeoff due merely to a drifting ground position estimate.
            self.emit(now, (0., 0., 0.))
            return
        if not t.armed or t.mode != "OFFBOARD":
            self.fault(now, f"已上鎖或離開 OFFBOARD（{t.mode}），不搶回模式")
            return
        self.air_seen = self.air_seen or t.landed == "AIR"
        if phase in {"HOLD", "ALIGN", "MOVE"} and t.landed != "AIR":
            self.fault(now, "空中追蹤的著陸狀態改變")
            return
        if phase in {"TAKING_OFF", "HOLD", "ALIGN", "MOVE", "LAND_BRAKE"}:
            error = math.dist(t.position, self.target)
        else:
            error = math.dist(t.position[:2], self.target[:2])
        if error > 0.5:
            if self.tracking_error_since is None:
                self.tracking_error_since = now
            if now - self.tracking_error_since > 1:
                self.fault(now, "追蹤誤差持續超過 0.5 m")
                return
        else:
            self.tracking_error_since = None
        if phase in {"TAKING_OFF", "MOVE"}:
            self.target = self.line.at(elapsed)
            done = (elapsed >= self.line.duration and math.dist(t.position, self.line.end) <= POSITION_TOL
                    and math.hypot(*t.velocity) <= SETTLED_SPEED and t.landed == "AIR"
                    and (phase == "TAKING_OFF" or abs(t.yaw) <= YAW_TOL))
            if self.is_settled(done, t.position_at, .5):
                self.enter("HOLD", now, "指令完成，等待下個指令（保持 Offboard）")
            elif elapsed > self.line.duration + 15:
                self.fault(now, "未在期限內抵達目標；不能只憑軌跡時間宣告完成")
                return
        elif phase == "ALIGN":
            remain = max(0., abs(self.start_yaw) - YAW_RATE * elapsed)
            self.yaw_ref = math.copysign(remain, self.start_yaw)
            if remain == 0 and abs(t.yaw) <= YAW_TOL:
                self.enter("MOVE", now, "heading 已對齊，開始直線移動")
            elif elapsed > abs(self.start_yaw) / YAW_RATE + 15:
                self.fault(now, "heading 未對齊")
                return
        elif phase == "LAND_BRAKE":
            if self.is_settled(math.hypot(*t.velocity) <= SETTLED_SPEED, t.position_at, .5):
                self.descent_started = now
                self.enter("DESCEND", now, "Offboard 下降中；等待 PX4 落地偵測")
            elif elapsed > 5:
                self.fault(now, "降落前未能穩定速度")
                return
        elif phase in {"DESCEND", "DISARMING"}:
            if now - self.land_started > self.landing_timeout:
                self.fault(now, "降落逾時，未強制上鎖")
                return
            # If takeoff was cancelled before AIR was ever observed, require
            # proximity to launch height IN ADDITION to fresh ground reports.
            plausible = self.air_seen or (self.ground_z is not None and abs(t.position[2] - self.ground_z) < .1)
            ground = (t.landed == "GROUND" and t.landed_at > self.descent_started
                      and math.hypot(*t.velocity) <= SETTLED_SPEED and plausible)
            if phase == "DESCEND" and self.is_settled(ground, t.landed_at, 1.):
                self.issue("DISARMING", "disarm", now)
            elif phase == "DISARMING":
                if not ground:
                    self.enter("DESCEND", now, "落地條件改變，取消上鎖重送，繼續確認")
                elif elapsed > 6:
                    self.fault(now, "已請求一般上鎖但未確認；沒有 force disarm")
                    return
                elif now - self.command_at >= .5:
                    self.send_command("disarm")
                    self.command_at = now
            self.emit(now, self.landing_velocity(now))
            return
        self.emit(now)
