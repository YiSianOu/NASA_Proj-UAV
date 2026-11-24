"""Every two seconds: mode, armed, phase, N/E, height relative to takeoff, yaw."""

import json
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Monitor(Node):
    def __init__(self):
        super().__init__("px4_states_monitor")
        self.create_subscription(String, "/flight/state", self.state, 10)
        self.latest = None
        self.last_rx = None
        self.previous_phase = None
        self.previous_ready_count = 0
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(2.0, self.print_state, clock=self.steady)

    def state(self, msg):
        try:
            state = json.loads(msg.data)
        except (ValueError, TypeError):
            self.latest = None
            return
        if not isinstance(state, dict):
            self.latest = None
            return
        self.latest = state
        self.last_rx = time.monotonic()
        phase = state.get("phase")
        ready_count = state.get("ready_count", 0)
        if isinstance(ready_count, int):
            if ready_count > self.previous_ready_count:
                print("ready：heading 已對齊，開始 XY 移動", flush=True)
            self.previous_ready_count = ready_count
        if phase == "HOLD" and self.previous_phase != "HOLD" and state.get("online"):
            print("已到達目標位置（未要求停穩）；可輸入新 XY 座標", flush=True)
        self.previous_phase = phase

    def print_state(self):
        state = self.latest or {}
        fresh = self.last_rx is not None and time.monotonic() - self.last_rx < 2.0
        if not fresh:
            state = {}
        warnings = list(state.get("warnings") or [])
        if not fresh:
            warnings.append("controller 狀態未更新；無法確認目前飛行狀態")
        mode = state.get("mode")
        if not isinstance(mode, str) or not mode:
            mode = "UNKNOWN"
        position = state.get("position_ned")
        x = y = altitude = None
        if (isinstance(position, (list, tuple)) and len(position) == 3
                and all(type(v) in (int, float) and math.isfinite(v) for v in position)):
            x, y, down = position
            altitude = state.get("relative_altitude")
        xy = state.get("xy_from_takeoff_target")
        origin_x, origin_y = xy if isinstance(xy, (list, tuple)) and len(xy) == 2 else (None, None)
        warning_text = "".join(f"[WARN] {warning}\n" for warning in warnings)
        print(f"PX4 mode: {mode}; armed: {state.get('armed', '--')}; phase: {state.get('phase', '--')}\n"
              f"position: x = {number(x)}, y = {number(y)}; altitude: {number(altitude)}\n"
              f"XY（起飛目標為原點）: x = {number(origin_x)}, y = {number(origin_y)}\n"
              f"yaw = {number(state.get('yaw_deg'))}; speed = {number(state.get('speed'))} m/s\n"
              f"tracking error = {number(state.get('tracking_error_m'))} m; goal error = {number(state.get('goal_error_m'))} m\n"
              f"{warning_text}"
              "-----------------------------------", flush=True)


def number(value):
    if type(value) in (int, float) and math.isfinite(value):
        return f"{value:.2f}"
    return "--"


def main():
    rclpy.init()
    node = None
    try:
        node = Monitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
