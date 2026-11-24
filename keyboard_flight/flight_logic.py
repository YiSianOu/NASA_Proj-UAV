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


def finite_vector(value):
    return (isinstance(value, (tuple, list)) and len(value) == 3
            and all(isinstance(v, (int, float)) and math.isfinite(v) for v in value))


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
                and now - self.estimator_at < 2 and self.estimate_ok and self.navigation_valid())

    def navigation_valid(self):
        """Usable last-known data; reception age is diagnostic, not a stop gate."""
        return (math.isfinite(self.heartbeat_at) and math.isfinite(self.position_at)
                and finite_vector(self.position) and finite_vector(self.velocity)
                and self.yaw is not None and math.isfinite(self.yaw))

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
        """Diagnostics, including warning-only reception ages; not Commander readiness."""
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
        if land_speed > 0.1:
            raise ValueError("此範例降落指令速度最多 0.1 m/s")
        self.t, self.send_command, self.send_target, self.report = telemetry, send_command, send_target, report
        self.takeoff_height, self.land_speed, self.landing_timeout = takeoff_height, land_speed, landing_timeout
        self.phase = "IDLE"
        self.changed_at = 0.
        self.target = self.yaw_ref = self.line = self.last_send = None
        self.velocity_ref = None
        self.pending = None
        self.seen = deque(maxlen=256)
        self.settled_from = None
        self.last_output = None
        self.data_paused_at = None
        self.frame_invalid = False
        self.notices = {}  # key -> (text, expiry); diagnostics never clear targets
        self.ready_count = 0
        self.ground_z = None
        self.takeoff_origin = None  # Fixed local-NED takeoff TARGET, not measured position.
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
        self.data_paused_at = None
        self.report(note)

    def clear_targets(self):
        self.target = self.yaw_ref = self.line = self.last_send = self.velocity_ref = None
        self.last_output = None
        self.data_paused_at = None

    def warn(self, key, text, now, ttl=10.):
        expiry = math.inf if ttl is None else now + ttl
        if key not in self.notices or self.notices[key][0] != text:
            self.report("警告：" + text)
        self.notices[key] = (text, expiry)

    def release_to_px4(self, now, note):
        """Stop commanding only after PX4 actually changed mode/disarmed."""
        self.clear_targets()
        self.enter("IDLE" if not self.t.armed else "PX4_CONTROL", now, note)

    def invalidate_frame(self, now):
        """No reset transform exists in LOCAL_POSITION_NED: never guess one."""
        active = self.phase not in {"IDLE", "PX4_CONTROL", "KILLED", "KILL_UNCONFIRMED"}
        self.clear_targets()
        self.takeoff_origin = self.ground_z = None
        self.frame_invalid = True
        self.warn("frame_reset", "定位時間戳疑似重置；舊軌跡與 XY 原點已清除。"
                  "控制中改送零速度；請降落後重新起飛建立原點", now, ttl=None)
        if active and not self.kill_latched:
            self.enter("FRAME_RESET", now, "持續送零速度，不沿用舊位置與 yaw 座標")

    def replay_last_output(self, now):
        """Keep transmitting finite commands without advancing a blind trajectory."""
        if self.last_output is not None:
            position, yaw, velocity = self.last_output
            self.send_target(position, yaw, velocity)
            self.velocity_ref = velocity
            self.last_send = now
        else:
            self.emit(now, (0., 0., 0.), ignore_yaw=True)

    def diagnostics(self, now):
        """Warnings never alter flight state or stop setpoint transmission."""
        t = self.t
        warnings = t.navigation_issues(now)
        self.notices = {k: v for k, v in self.notices.items() if now < v[1]}
        warnings.extend(text for text, expiry in self.notices.values())
        elapsed = now - self.changed_at
        if self.data_paused_at is not None:
            warnings.append("資料無效：暫停軌跡推進，重送最後有效命令（速度命令不代表懸停）")
        if self.phase == "LAND_BRAKE" and elapsed > 5:
            warnings.append("降落前尚未穩定速度，繼續保持位置；不因等待逾時斷流")
        if self.phase in {"LAND_BRAKE", "DESCEND", "DISARMING"} and now - self.land_started > self.landing_timeout:
            warnings.append("降落等待超過設定時間；繼續目前控制，不強制上鎖、不斷流")
        if self.phase in {"DISARMING", "GROUND_STOP"} and elapsed > 6:
            warnings.append("尚未確認 PX4 上鎖；持續監看與送目標，不因等待逾時斷流")
        tracking_error = goal_error = None
        if finite_vector(t.position):
            if self.target is not None:
                tracking_error = (math.dist(t.position[:2], self.target[:2])
                                  if self.phase in {"DESCEND", "DISARMING"}
                                  else math.dist(t.position, self.target))
            if self.line is not None:
                goal_error = math.dist(t.position, self.line.end)
        position_fresh = now - t.position_at < .5
        if position_fresh and tracking_error is not None and tracking_error > POSITION_TOL:
            warnings.append(f"追蹤誤差 {tracking_error:.3f} m > {POSITION_TOL:.2f} m（只警告，持續串流）")
        checking_arrival = (self.phase == "HOLD" or
                            (self.phase in {"TAKING_OFF", "MOVE"} and self.line is not None
                             and (now - self.changed_at >= self.line.duration
                                  or (goal_error is not None and goal_error <= POSITION_TOL))))
        if (checking_arrival and position_fresh and t.velocity is not None
                and all(math.isfinite(v) for v in t.velocity)
                and math.hypot(*t.velocity) > SETTLED_SPEED):
            warnings.append(f"到點速度 {math.hypot(*t.velocity):.3f} m/s > {SETTLED_SPEED:.2f} m/s（只警告）")
        return dict(warnings=warnings, tracking_error_m=tracking_error, goal_error_m=goal_error,
                    data_hold=self.data_paused_at is not None, frame_valid=not self.frame_invalid)

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
            if not accepted:
                self.warn("ack_" + command, f"PX4 拒絕 {command}：{result}；仍以 armed 回報確認結果", now)
            else:
                self.notices.pop("ack_" + command, None)
        elif accepted:
            self.notices.pop("ack_" + command, None)
            self.report(f"{command}：{result}；仍等待遙測確認")
        else:
            self.warn("ack_" + command, f"PX4 拒絕 {command}：{result}；保留既有串流，等待狀態或新操作",
                      now, ttl=None)

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
        if not math.isfinite(self.t.heartbeat_at):
            raise ValueError("PX4 heartbeat 尚未就緒")
        if mode == "land":
            self.request_land(now)
            return
        if not self.t.navigation_valid():
            raise ValueError("遙測檢查未通過：" + "；".join(self.t.navigation_issues(now)))
        t = self.t
        if mode == "arm":
            if self.phase not in {"IDLE", "PX4_CONTROL"} or t.armed or t.landed != "GROUND":
                raise ValueError("0 僅在地面未解鎖、無進行中操作時接受")
            self.landing_setup_ok(now)
            self.clear_targets()
            self.target, self.yaw_ref = t.position, t.yaw
            self.ground_z, self.air_seen = None, False
            self.takeoff_origin = None
            self.frame_invalid = False
            self.notices.clear()
            self.enter("PRIME", now, "預送零速度目標；尚未解鎖、尚未起飛")
            return
        if mode == "takeoff":
            if self.phase != "ARMED_GROUND" or not t.armed or t.mode != "OFFBOARD" or t.landed != "GROUND":
                raise ValueError("先按 0，等待 ARMED_GROUND；若已自動上鎖需重新按 0")
            self.landing_setup_ok(now)
            height = request.get("height", self.takeoff_height)
            if type(height) not in (int, float) or not math.isfinite(height) or height <= 0:
                raise ValueError("起飛高度必須是大於 0 m 的有限數值")
            line = Line(t.position, (0., 0., height))
            self.target, self.yaw_ref = t.position, t.yaw
            self.ground_z = t.position[2]
            self.line = line
            self.takeoff_origin = line.end
            self.air_seen = False
            self.enter("TAKING_OFF", now, f"Offboard 起飛：相對本次起點上升 {height:.2f} m")
            return
        if mode != "offboard":
            raise ValueError("不明模式")
        values = [request.get(k) for k in "xy"]
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError("x/y 必須是相對起飛目標原點的有限座標，單位公尺")
        # Accept legacy z=0 messages, but never silently discard a height change.
        if "z" in request and (type(request["z"]) not in (int, float) or request["z"] != 0):
            raise ValueError("Offboard 僅接受 XY 座標，Z 固定為起飛高度；請移除 z 或填 0")
        if (self.phase not in {"TAKING_OFF", "HOLD", "ALIGN", "MOVE"}
                or t.mode != "OFFBOARD" or not t.armed or t.landed != "AIR"):
            raise ValueError("2 需要已解鎖、OFFBOARD、AIR，且不在降落或故障流程")
        if self.takeoff_origin is None:
            raise ValueError("尚未建立起飛目標原點")
        # Absolute XY coordinates in the takeoff-target frame; never accumulate.
        # XY starts at the current reference; Z is the fixed takeoff target plane.
        # If takeoff is interrupted after AIR, retain its final altitude target.
        endpoint = Line(self.takeoff_origin, (*values, 0.)).end
        start = (self.target[0], self.target[1], self.takeoff_origin[2])
        offset = (endpoint[0] - start[0], endpoint[1] - start[1], 0.)
        line = Line(start, offset)
        if self.ground_z is None or self.ground_z - line.end[2] < 0.15:
            raise ValueError("追蹤目標過低；請用 3 執行降落")
        self.line, self.start_yaw = line, self.yaw_ref
        self.target = start
        self.enter("ALIGN", now, "對齊 heading 0°；對齊後直接移動")

    def request_land(self, now):
        t = self.t
        if self.phase in {"LAND_BRAKE", "DESCEND", "DISARMING"}:
            return
        if self.phase == "GROUND_STOP" and (not t.armed or t.landed == "GROUND"):
            return
        # Cancel the arm sequence as well. Repeated NORMAL disarms cover an
        # in-flight arm command without confusing its ACK with stop completion.
        if self.phase in {"PRIME", "SELECT_OFFBOARD", "ARMING", "ARMED_GROUND"}:
            if t.landed != "GROUND":
                raise ValueError("無法確認在地面；不執行一般上鎖")
            self.issue("GROUND_STOP", "disarm", now)
            return
        if not t.armed:
            raise ValueError("飛機已上鎖，沒有降落動作")
        if not t.navigation_valid() or t.mode != "OFFBOARD":
            raise ValueError("Offboard 降落需要有效本地定位及目前為 OFFBOARD；查看 PX4 保護狀態")
        self.landing_setup_ok(now)
        self.target, self.yaw_ref = t.position, t.yaw
        self.line = None
        self.air_seen = self.air_seen or t.landed == "AIR"
        self.land_started = now
        self.enter("LAND_BRAKE", now, "取消移動，先穩定位置，再用 Offboard 下降")

    def is_settled(self, condition, stamp, seconds):
        if not condition:
            self.settled_from = None
        elif self.settled_from is None:
            self.settled_from = stamp
        return self.settled_from is not None and stamp - self.settled_from >= seconds

    def emit(self, now, velocity=None, ignore_yaw=False):
        position = None if velocity is not None else self.target
        yaw = None if ignore_yaw else self.yaw_ref
        self.send_target(position, yaw, velocity)
        self.last_output = (position, yaw, velocity)
        self.velocity_ref = velocity
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
        if phase in {"IDLE", "PX4_CONTROL"}:
            return
        elapsed = now - self.changed_at

        # An actual PX4 mode/disarm report owns control. Never reclaim it.
        # PRIME/SELECT_OFFBOARD intentionally stream before mode selection/arming.
        heartbeat_known = math.isfinite(t.heartbeat_at)
        if heartbeat_known and phase not in {"PRIME", "SELECT_OFFBOARD"} and t.mode != "OFFBOARD":
            self.release_to_px4(now, f"PX4 目前為 {t.mode}；交還控制，不搶回 Offboard")
            return
        if (heartbeat_known and phase not in {"PRIME", "SELECT_OFFBOARD", "ARMING", "GROUND_STOP"}
                and not t.armed):
            self.release_to_px4(now, "PX4 已回報上鎖；停止送目標，不自動重新解鎖")
            return

        if phase == "FRAME_RESET":
            self.emit(now, (0., 0., 0.), ignore_yaw=True)
            return

        if phase == "GROUND_STOP":
            ground = now - t.landed_at < 1 and t.landed == "GROUND"
            if self.is_settled(ground and now - t.heartbeat_at < 3 and not t.armed
                               and t.heartbeat_at > self.changed_at, t.heartbeat_at, 1.) and elapsed >= 2:
                self.release_to_px4(now, "已取消操作，PX4 已回報上鎖")
                return
            if not ground:
                self.warn("ground_stop", "取消解鎖時未確認在地面；不重送一般上鎖，保留零速度串流", now)
            elif now - self.command_at >= .5:
                self.send_command("disarm")
                self.command_at = now
            self.emit(now, (0., 0., 0.), ignore_yaw=self.yaw_ref is None)
            return

        ground_sequence = phase in {"PRIME", "SELECT_OFFBOARD", "ARMING", "ARMED_GROUND"}
        if ground_sequence:
            if t.landed != "GROUND":
                self.warn("ground_state", "地面流程回報非 GROUND；只警告，不退出控制器", now)
            if phase in {"PRIME", "SELECT_OFFBOARD"} and t.armed:
                self.warn("external_arm", "偵測到外部解鎖，依 PX4 實際模式處理，不退出控制器", now)
                if t.mode != "OFFBOARD":
                    self.release_to_px4(now, f"外部解鎖後 PX4 為 {t.mode}；不搶回模式")
                    return
                self.enter("ARMED_GROUND" if t.landed == "GROUND" else "WAIT_USER", now,
                           "已解鎖：維持零速度，等待使用者操作")
            elif phase == "PRIME" and elapsed >= 1.2 and self.last_send is not None:
                self.issue("SELECT_OFFBOARD", "offboard", now)
            elif phase == "SELECT_OFFBOARD":
                if t.mode == "OFFBOARD" and t.heartbeat_at > self.changed_at:
                    self.notices.pop("ack_offboard", None)
                    self.issue("ARMING", "arm", now)
            elif phase == "ARMING":
                if t.armed and t.heartbeat_at > self.changed_at:
                    self.notices.pop("ack_arm", None)
                    self.enter("ARMED_GROUND" if t.landed == "GROUND" else "WAIT_USER", now,
                               "已解鎖：維持零速度，等待使用者操作")
            elif phase == "ARMED_GROUND" and t.landed != "GROUND":
                self.enter("WAIT_USER", now, "離地狀態改變：保留零速度串流，可請求降落")
            self.emit(now, (0., 0., 0.), ignore_yaw=self.yaw_ref is None)
            return
        if phase == "WAIT_USER":
            self.emit(now, (0., 0., 0.), ignore_yaw=self.yaw_ref is None)
            return

        if not t.navigation_valid():
            if self.data_paused_at is None:
                self.data_paused_at = now
            self.replay_last_output(now)
            return
        if self.data_paused_at is not None:
            paused = now - self.data_paused_at
            self.changed_at += paused
            if phase in {"DESCEND", "DISARMING"}:
                self.descent_started += paused
            self.data_paused_at = None
            elapsed = now - self.changed_at

        self.air_seen = self.air_seen or t.landed == "AIR"
        if phase in {"HOLD", "ALIGN", "MOVE"} and t.landed != "AIR":
            self.warn("air_state", "空中流程回報非 AIR；只警告，持續既有目標", now)
        else:
            self.notices.pop("air_state", None)

        if phase in {"TAKING_OFF", "MOVE"}:
            self.target = self.line.at(elapsed)
            done = (now - t.position_at < .5 and t.position_at > self.changed_at
                    and now - t.landed_at < 1 and t.landed == "AIR"
                    and math.dist(t.position, self.line.end) <= POSITION_TOL
                    and (phase == "TAKING_OFF" or
                         (now - t.attitude_at < .5 and abs(t.yaw) <= YAW_TOL)))
            if done:
                self.target = self.line.end
                self.enter("HOLD", now, "已到達目標位置（未要求停穩）；保持 Offboard，可輸入新指令")
        elif phase == "ALIGN":
            remain = max(0., abs(self.start_yaw) - YAW_RATE * elapsed)
            self.yaw_ref = math.copysign(remain, self.start_yaw)
            if remain == 0 and now - t.attitude_at < .5 and abs(t.yaw) <= YAW_TOL:
                self.ready_count += 1
                self.enter("MOVE", now, "ready：heading 已對齊，開始直線移動")
        elif phase == "LAND_BRAKE":
            if self.is_settled(now - t.position_at < .5 and math.hypot(*t.velocity) <= SETTLED_SPEED,
                               t.position_at, .5):
                self.descent_started = now
                self.enter("DESCEND", now, "Offboard 下降中；等待 PX4 落地偵測")
        elif phase in {"DESCEND", "DISARMING"}:
            plausible = (self.air_seen or self.frame_invalid or
                         (self.ground_z is not None and abs(t.position[2] - self.ground_z) < .1))
            ground = (now - t.landed_at < 1 and now - t.position_at < .5
                      and t.landed == "GROUND" and t.landed_at > self.descent_started
                      and math.hypot(*t.velocity) <= SETTLED_SPEED and plausible)
            if phase == "DESCEND" and self.is_settled(ground, t.landed_at, 1.):
                self.issue("DISARMING", "disarm", now)
            elif phase == "DISARMING":
                if not ground:
                    self.enter("DESCEND", now, "落地條件改變，取消上鎖重送，繼續確認")
                elif now - self.command_at >= .5:
                    self.send_command("disarm")
                    self.command_at = now
            self.emit(now, self.landing_velocity(now))
            return
        self.emit(now)
