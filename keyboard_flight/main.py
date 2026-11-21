"""ROS 2 Humble node owning the MAVLink connection for keyboard-commanded flight."""

import json
import math
import time

import rclpy
from pymavlink import mavutil
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from .flight_logic import Flight, Telemetry

M = mavutil.mavlink
COMMANDS = {"arm": M.MAV_CMD_COMPONENT_ARM_DISARM,
            "offboard": M.MAV_CMD_DO_SET_MODE, "disarm": M.MAV_CMD_COMPONENT_ARM_DISARM,
            "kill": M.MAV_CMD_COMPONENT_ARM_DISARM}
AUTO_MODES = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL", 6: "LAND",
              8: "FOLLOW", 9: "PRECLAND"}
MAIN_MODES = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED"}
LANDED = {0: "UNKNOWN", 1: "GROUND", 2: "AIR", 3: "TAKING_OFF", 4: "LANDING"}
RATES = ((M.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20), (M.MAVLINK_MSG_ID_ATTITUDE, 20),
         (M.MAVLINK_MSG_ID_ESTIMATOR_STATUS, 4), (M.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 4))
POSITION_YAW_MASK = (M.POSITION_TARGET_TYPEMASK_VX_IGNORE | M.POSITION_TARGET_TYPEMASK_VY_IGNORE
                     | M.POSITION_TARGET_TYPEMASK_VZ_IGNORE | M.POSITION_TARGET_TYPEMASK_AX_IGNORE
                     | M.POSITION_TARGET_TYPEMASK_AY_IGNORE | M.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                     | M.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
VELOCITY_YAW_MASK = (M.POSITION_TARGET_TYPEMASK_X_IGNORE | M.POSITION_TARGET_TYPEMASK_Y_IGNORE
                     | M.POSITION_TARGET_TYPEMASK_Z_IGNORE | M.POSITION_TARGET_TYPEMASK_AX_IGNORE
                     | M.POSITION_TARGET_TYPEMASK_AY_IGNORE | M.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                     | M.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
CHECK_PARAMETERS = ("LNDMC_Z_VEL_MAX", "MPC_Z_VEL_MAX_DN")


class Controller(Node):
    def __init__(self):
        super().__init__("keyboard_flight_controller")
        port = self.declare_parameter("port", "/dev/ttyUSB0").value
        baud = self.declare_parameter("baud", 57600).value
        self.system = self.declare_parameter("target_system", 1).value
        self.source = self.declare_parameter("source_system", 245).value
        if not (1 <= self.source <= 255 and 1 <= self.system <= 255 and self.source != self.system):
            raise ValueError("source_system 與 target_system 需為相異的 1..255")
        self.t = Telemetry()
        self.events = self.create_publisher(String, "/flight/event", 20)
        self.states = self.create_publisher(String, "/flight/state", 10)
        self.last_note = "等待 PX4"
        self.flight = Flight(
            self.t, self.command, self.send_target, self.event,
            takeoff_height=float(self.declare_parameter("takeoff_height", 0.3).value),
            land_speed=float(self.declare_parameter("land_speed", 0.1).value),
            landing_timeout=float(self.declare_parameter("landing_timeout", 120.0).value))
        self.create_subscription(String, "/flight/request", self.request,
                                 QoSProfile(depth=10, lifespan=Duration(seconds=2)))
        self.link = mavutil.mavlink_connection(port, baud=baud, source_system=self.source,
                                              source_component=M.MAV_COMP_ID_ONBOARD_COMPUTER)
        if hasattr(getattr(self.link, "port", None), "write_timeout"):
            self.link.port.write_timeout = 0.05
        self.boot_ms = None
        self.next_heartbeat = self.next_rates = self.next_state = 0.0
        self.next_parameters = 0.0
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / 50, self.tick, clock=self.steady)
        self.get_logger().info(f"等待 PX4 sysid {self.system} 及 /flight/request")

    def event(self, text):
        self.last_note = text
        self.events.publish(String(data=text))
        self.get_logger().info(text)

    def request(self, msg):
        try:
            self.flight.request(json.loads(msg.data), time.monotonic())
        except (ValueError, OverflowError) as exc:
            self.event(f"指令未執行：{exc}")

    def command(self, name):
        # Normal operation never sends NAV_TAKEOFF / NAV_LAND or force-arm.
        params = dict(arm=(1, 0, 0, 0, 0, 0, 0),
                      disarm=(0, 0, 0, 0, 0, 0, 0),
                      kill=(0, 21196, 0, 0, 0, 0, 0),
                      offboard=(M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, 0, 0, 0))
        self.link.mav.command_long_send(self.system, M.MAV_COMP_ID_AUTOPILOT1,
                                        COMMANDS[name], 0, *params[name])

    def send_target(self, position, yaw, velocity=None):
        now = time.monotonic()
        stamp = (self.boot_ms + int(1000 * (now - self.t.position_at))) & 0xffffffff
        mask = POSITION_YAW_MASK if velocity is None else VELOCITY_YAW_MASK
        p = position if velocity is None else (0., 0., 0.)
        v = (0., 0., 0.) if velocity is None else velocity
        self.link.mav.set_position_target_local_ned_send(
            stamp, self.system, M.MAV_COMP_ID_AUTOPILOT1, M.MAV_FRAME_LOCAL_NED,
            mask, *p, *v, 0., 0., 0., yaw, 0.)

    def receive(self, msg, now):
        if (msg.get_srcSystem(), msg.get_srcComponent()) != (self.system, M.MAV_COMP_ID_AUTOPILOT1):
            return
        t = self.t
        kind = msg.get_type()
        if kind == "HEARTBEAT" and msg.autopilot == M.MAV_AUTOPILOT_PX4:
            main = (msg.custom_mode >> 16) & 255
            sub = (msg.custom_mode >> 24) & 255
            if not msg.base_mode & M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
                t.mode = "UNKNOWN"
            elif main == 4:
                t.mode = "AUTO." + AUTO_MODES.get(sub, str(sub))
            else:
                t.mode = MAIN_MODES.get(main, str(main))
            t.heartbeat_at = now
            t.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
        elif kind == "LOCAL_POSITION_NED":
            timestamp = int(msg.time_boot_ms)
            if self.boot_ms is not None:
                delta = (timestamp - self.boot_ms) & 0xffffffff
                if delta == 0:
                    return
                if delta >= 0x80000000:
                    self.flight.fault(now, "飛控定位時間重置")
                    t.heartbeat_at = t.landed_at = t.estimator_at = t.attitude_at = -math.inf
            pos = (msg.x, msg.y, msg.z)
            vel = (msg.vx, msg.vy, msg.vz)
            self.boot_ms = timestamp
            if all(map(math.isfinite, (*pos, *vel))):
                if (self.flight.target is not None and t.position is not None
                        and math.dist(pos, t.position) > 0.3 and now - t.position_at < 0.5):
                    self.flight.fault(now, "本地定位跳變，停止外部目標串流")
                t.position, t.velocity, t.position_at = pos, vel, now
            else:
                t.position = t.velocity = None
        elif kind == "ATTITUDE":
            if (self.flight.target is not None and t.yaw is not None and math.isfinite(msg.yaw)
                    and now - t.attitude_at < .5
                    and abs(math.remainder(msg.yaw - t.yaw, 2 * math.pi)) > math.radians(30)):
                self.flight.fault(now, "heading 突然跳變")
            t.yaw = math.remainder(msg.yaw, 2 * math.pi) if math.isfinite(msg.yaw) else None
            t.attitude_at = now
        elif kind == "EXTENDED_SYS_STATE":
            t.landed = LANDED.get(msg.landed_state, "UNKNOWN")
            t.landed_at = now
        elif kind == "ESTIMATOR_STATUS":
            t.estimator_flags = int(msg.flags)
            required = (M.ESTIMATOR_ATTITUDE | M.ESTIMATOR_POS_VERT_ABS
                        | M.ESTIMATOR_VELOCITY_HORIZ | M.ESTIMATOR_VELOCITY_VERT)
            horizontal = M.ESTIMATOR_POS_HORIZ_REL | M.ESTIMATOR_POS_HORIZ_ABS
            # PX4 v1.17 also sets CONST_POS_MODE for vehicle_at_rest. It is
            # not by itself a navigation fault on the ground. All required
            # validity bits and the acceleration-error check still apply.
            ground_fresh = t.landed == "GROUND" and 0 <= now - t.landed_at < 1.
            bad = M.ESTIMATOR_ACCEL_ERROR
            if not ground_fresh:
                bad |= M.ESTIMATOR_CONST_POS_MODE
            t.estimate_ok = (msg.flags & required == required and bool(msg.flags & horizontal)
                             and not bool(msg.flags & bad))
            t.estimator_at = now
        elif kind == "PARAM_VALUE":
            name = msg.param_id.decode("ascii", errors="ignore") if isinstance(msg.param_id, bytes) else msg.param_id
            name = name.rstrip("\x00")
            if name in CHECK_PARAMETERS and msg.param_type == M.MAV_PARAM_TYPE_REAL32:
                t.parameters[name] = (float(msg.param_value), now)
        elif kind == "COMMAND_ACK":
            pending = self.flight.pending
            if (pending and msg.command == COMMANDS[pending]
                    and getattr(msg, "target_system", 0) in (0, self.source)
                    and getattr(msg, "target_component", 0) in (0, M.MAV_COMP_ID_ONBOARD_COMPUTER)):
                label = M.enums["MAV_RESULT"].get(msg.result)
                self.flight.acknowledge(pending, msg.result in (M.MAV_RESULT_ACCEPTED, M.MAV_RESULT_IN_PROGRESS),
                                        label.name if label else str(msg.result), now)
        elif kind == "STATUSTEXT" and msg.severity <= M.MAV_SEVERITY_WARNING:
            self.event(f"PX4：{msg.text}")

    def snapshot(self, now):
        t = self.t
        online = now - t.heartbeat_at < 3
        position = t.position if now - t.position_at < 0.5 else None
        state = dict(phase=self.flight.phase, mode=t.mode if online else "UNKNOWN",
                     armed=t.armed if online else None,
                     landed=t.landed if now - t.landed_at < 2 else "UNKNOWN",
                     yaw_deg=math.degrees(t.yaw) if t.yaw is not None and now - t.attitude_at < 0.5 else None,
                     speed=math.hypot(*t.velocity) if position is not None else None,
                     position_ned=position,
                     target_ned=self.flight.target if self.flight.velocity_ref is None else None,
                     velocity_target_ned=self.flight.velocity_ref,
                     navigation_ok=t.navigation_ok(now),
                     navigation_issues=t.navigation_issues(now),
                     telemetry_age_s=t.telemetry_age_s(now),
                     estimator_flags=t.estimator_flags, estimate_ok=t.estimate_ok,
                     relative_altitude=(self.flight.ground_z - position[2]
                                        if position is not None and self.flight.ground_z is not None else None),
                     online=online, note=self.last_note)
        self.states.publish(String(data=json.dumps(state, ensure_ascii=False, allow_nan=False)))

    def tick(self):
        deadline = time.monotonic() + 0.003
        while time.monotonic() < deadline:
            msg = self.link.recv_match(blocking=False)
            if msg is None:
                break
            self.receive(msg, time.monotonic())
        now = time.monotonic()
        if now >= self.next_heartbeat:
            self.link.mav.heartbeat_send(M.MAV_TYPE_ONBOARD_CONTROLLER, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.next_heartbeat = now + 1
        if now - self.t.heartbeat_at < 3 and now >= self.next_rates:
            for message, hz in RATES:
                self.link.mav.command_long_send(self.system, M.MAV_COMP_ID_AUTOPILOT1,
                                                M.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                                message, 1e6 / hz, 0, 0, 0, 0, 0)
            self.next_rates = now + 10
        if now - self.t.heartbeat_at < 3 and now >= self.next_parameters:
            for name in CHECK_PARAMETERS:
                self.link.mav.param_request_read_send(self.system, M.MAV_COMP_ID_AUTOPILOT1,
                                                     name.encode("ascii"), -1)
            self.next_parameters = now + 5
        self.flight.tick(now)
        if now >= self.next_state:
            self.snapshot(now)
            self.next_state = now + 0.1


def main():
    rclpy.init()
    node = None
    try:
        node = Controller()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.link.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
