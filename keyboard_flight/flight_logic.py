"""Keyboard flight state machine. Positions and yaw use PX4 local NED and radians."""

import math
from collections import deque
from dataclasses import dataclass


MAX_SPEED = 0.1  # metres/second, Euclidean norm of the Offboard reference velocity
YAW_RATE = math.radians(15)
YAW_TOL = math.radians(5)
POSITION_TOL = 0.05
SETTLED_SPEED = 0.05
SETTLE_SECONDS = 0.5


@dataclass
class Telemetry:
    mode: str = "UNKNOWN"
    armed: bool = False
    landed: str = "UNKNOWN"
    position: tuple | None = None
    velocity: tuple | None = None
    yaw: float | None = None
    estimate_ok: bool = False
    heartbeat_at: float = -math.inf
    landed_at: float = -math.inf
    position_at: float = -math.inf
    attitude_at: float = -math.inf
    estimator_at: float = -math.inf

    def navigation_ok(self, now):
        return (now - self.heartbeat_at < 3 and now - self.landed_at < 2
                and now - self.position_at < 0.5 and now - self.attitude_at < 0.5
                and now - self.estimator_at < 2 and self.estimate_ok
                and self.position is not None and self.velocity is not None
                and self.yaw is not None)


class Line:
    """One half-cosine displacement applied simultaneously to all three axes."""

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
        fraction = math.sin(math.pi * max(0.0, elapsed) / (2 * self.duration)) ** 2
        return tuple(a + fraction * b for a, b in zip(self.start, self.delta))


class Flight:
    def __init__(self, telemetry, send_command, send_target, report):
        self.t = telemetry
        self.send_command = send_command
        self.send_target = send_target
        self.report = report
        self.phase = "IDLE"
        self.changed_at = 0.0
        self.target = None
        self.yaw_ref = None
        self.line = None
        self.start_yaw = 0.0
        self.settled_from = None
        self.last_send = None
        self.pending = None
        self.seen = deque(maxlen=256)
        self.tracking_error_since = None

    def enter(self, phase, now, note):
        self.phase = phase
        self.changed_at = now
        self.settled_from = None
        self.pending = None
        self.report(note)

    def fault(self, now, reason):
        self.enter("FAULT", now, reason)
        self.target = self.yaw_ref = self.last_send = None

    def issue(self, phase, command, now):
        self.enter(phase, now, f"已請求 {command}，等待 PX4 回報")
        self.pending = command
        self.send_command(command)

    def acknowledge(self, command, accepted, result, now):
        if command == self.pending:
            if accepted:
                self.report(f"{command}：{result}，等待遙測確認")
            else:
                if command == "land":
                    self.land_unconfirmed(now, f"PX4 拒絕 land：{result}")
                else:
                    self.fault(now, f"PX4 拒絕 {command}：{result}")

    def land_unconfirmed(self, now, reason):
        if self.t.mode == "OFFBOARD" and self.target is not None and self.t.navigation_ok(now):
            self.enter("LAND_RETRY_HOLD", now, reason + "；維持目前目標，可重送 land")
        else:
            self.fault(now, reason)

    def request(self, request, now):
        if not isinstance(request, dict):
            raise ValueError("請求必須是 JSON 物件")
        request_id = request.get("id")
        if not isinstance(request_id, str) or not 0 < len(request_id) <= 128:
            raise ValueError("缺少有效的指令 id")
        if request_id in self.seen:
            return
        self.seen.append(request_id)
        mode = request.get("mode")
        if now - self.t.heartbeat_at >= 3:
            raise ValueError("PX4 heartbeat 尚未就緒")
        if mode == "land":
            if self.phase in {"LAND_REQUEST", "LANDING"}:
                return
            if not self.t.armed and self.phase not in {"SELECT_TAKEOFF", "ARMING"}:
                raise ValueError("飛機尚未解鎖")
            self.issue("LAND_REQUEST", "land", now)
            return
        if not self.t.navigation_ok(now):
            raise ValueError("等待有效的定位、速度、姿態與著陸狀態")
        if mode == "takeoff":
            if self.phase not in {"IDLE", "FAULT"} or self.t.armed or self.t.landed != "GROUND":
                raise ValueError("takeoff 僅在地面、未解鎖且沒有進行中指令時接受")
            # PX4 v1.17 NAV_TAKEOFF selects the autonomous mode; normal arming
            # then runs the checks for that mode, even when there is no RC.
            self.issue("SELECT_TAKEOFF", "takeoff", now)
            return
        if mode != "offboard":
            raise ValueError("不明模式")
        values = [request.get(k) for k in ("x", "y", "z")]
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError("x/y/z 都必須是有限的公尺數")
        if not self.t.armed or self.t.landed != "AIR":
            raise ValueError("請等待起飛完成")
        if self.phase == "HOLD" and self.t.mode == "OFFBOARD":
            # Reuse the previous endpoint so the reference stays continuous.
            if (math.dist(self.t.position, self.target) > POSITION_TOL
                    or math.hypot(*self.t.velocity) > SETTLED_SPEED):
                raise ValueError("尚未穩定保持在上一個端點")
            start = self.target
        elif self.phase in {"IDLE", "AIRBORNE", "FAULT"} and self.t.mode == "AUTO.LOITER":
            if math.hypot(*self.t.velocity) > 0.1:
                raise ValueError("等待 PX4 在 Hold 中穩定懸停")
            start = self.t.position
        else:
            raise ValueError(f"目前 {self.phase} / {self.t.mode}，未接受新位移")
        line = Line(start, values)
        self.line = line
        self.tracking_error_since = None
        self.target = start
        if self.phase == "HOLD":
            self.start_yaw = self.yaw_ref
            self.enter("ALIGN", now, "銜接新位移，確認 heading")
        else:
            self.start_yaw = self.yaw_ref = self.t.yaw
            self.last_send = None
            self.enter("PRIME", now, "預送懸停目標")

    def tick(self, now):
        phase, t = self.phase, self.t
        if phase in {"IDLE", "FAULT"}:
            return
        if now - t.heartbeat_at >= 3:
            self.fault(now, "PX4 heartbeat 逾時；待重新確認飛行狀態")
            return
        elapsed = now - self.changed_at
        if phase in {"LAND_REQUEST", "LANDING"}:
            if phase == "LANDING" and now - t.landed_at < 2 and t.landed == "GROUND" and not t.armed:
                self.target = self.yaw_ref = self.last_send = None
                self.enter("IDLE", now, "已在地面並上鎖")
            elif phase == "LAND_REQUEST" and (t.mode == "AUTO.LAND"
                                              or (now - t.landed_at < 2 and t.landed == "LANDING")):
                self.target = self.yaw_ref = self.last_send = None
                self.enter("LANDING", now, "PX4 接手降落")
            elif phase == "LAND_REQUEST" and elapsed > 5:
                self.land_unconfirmed(now, "未確認 Land，請查看 PX4 模式")
            elif phase == "LAND_REQUEST" and self.target is not None and t.mode == "OFFBOARD" and t.navigation_ok(now):
                self.send_target(self.target, self.yaw_ref)
                self.last_send = now
            elif phase == "LANDING" and t.mode != "AUTO.LAND" and t.landed == "AIR":
                self.fault(now, f"降落期間模式改為 {t.mode}")
            return
        if phase in {"SELECT_TAKEOFF", "ARMING", "TAKING_OFF", "AIRBORNE"}:
            if phase == "SELECT_TAKEOFF" and t.mode == "AUTO.TAKEOFF" and not t.armed:
                if t.navigation_ok(now) and t.landed == "GROUND":
                    self.issue("ARMING", "arm", now)
                else:
                    self.fault(now, "解鎖前狀態改變")
            elif phase == "SELECT_TAKEOFF" and t.armed and t.mode == "AUTO.TAKEOFF":
                self.enter("TAKING_OFF", now, "PX4 已解鎖，自動起飛中")
            elif phase == "ARMING" and t.armed and t.mode == "AUTO.TAKEOFF":
                self.enter("TAKING_OFF", now, "PX4 已解鎖，自動起飛中")
            elif phase in {"ARMING", "TAKING_OFF"} and t.armed and t.landed == "AIR" and t.mode == "AUTO.LOITER":
                self.enter("AIRBORNE", now, "起飛完成，可輸入 offboard")
            elif phase in {"SELECT_TAKEOFF", "ARMING"} and elapsed > 6:
                self.fault(now, "未確認起飛模式或解鎖，請查看 PX4 模式與訊息")
            elif phase == "TAKING_OFF" and (not t.armed or t.mode not in {"AUTO.TAKEOFF", "AUTO.LOITER"}):
                self.fault(now, f"起飛中斷，PX4 模式 {t.mode}")
            elif phase == "AIRBORNE" and not t.armed:
                self.enter("IDLE", now, "PX4 已上鎖")
            return
        if not t.navigation_ok(now) or not t.armed or t.landed != "AIR":
            self.fault(now, "Offboard 所需遙測失效，停止外部目標串流")
            return
        if self.last_send is not None and now - self.last_send > 0.4:
            self.fault(now, "Offboard 目標串流逾時")
            return
        if math.dist(t.position, self.target) > 1.0:
            if self.tracking_error_since is None:
                self.tracking_error_since = now
            if now - self.tracking_error_since >= 1.0:
                self.fault(now, "持續追蹤誤差超過 1 m，停止外部目標串流")
                return
        else:
            self.tracking_error_since = None
        if phase in {"PRIME", "ENTER_OFFBOARD"}:
            if phase == "ENTER_OFFBOARD" and t.mode == "OFFBOARD":
                self.enter("ALIGN", now, "已切入 Offboard，對齊 heading 0°")
            elif t.mode != "AUTO.LOITER":
                self.fault(now, f"PX4 模式意外變為 {t.mode}")
                return
            elif phase == "PRIME" and elapsed >= 1.2 and self.last_send is not None:
                self.issue("ENTER_OFFBOARD", "offboard", now)
            elif phase == "ENTER_OFFBOARD" and elapsed > 5:
                self.fault(now, "未確認切入 Offboard")
                return
        elif t.mode != "OFFBOARD":
            self.fault(now, f"PX4 已離開 Offboard：{t.mode}")
            return
        elif phase == "ALIGN":
            remaining = max(0.0, abs(self.start_yaw) - YAW_RATE * elapsed)
            self.yaw_ref = math.copysign(remaining, self.start_yaw)
            if remaining == 0.0 and abs(t.yaw) <= YAW_TOL:
                self.enter("MOVE", now, "heading 已對齊，開始直線平移")
        elif phase == "MOVE":
            self.target = self.line.at(elapsed)
            at_end = (elapsed >= self.line.duration and math.dist(t.position, self.line.end) <= POSITION_TOL
                      and math.hypot(*t.velocity) <= SETTLED_SPEED and abs(t.yaw) <= YAW_TOL)
            if not at_end:
                self.settled_from = None
            elif self.settled_from is None:
                self.settled_from = t.position_at
            elif t.position_at - self.settled_from >= SETTLE_SECONDS:
                self.enter("HOLD", now, "指令完成，等待下個指令")
        self.send_target(self.target, self.yaw_ref)
        self.last_send = now
