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
        if phase == "HOLD" and self.previous_phase != "HOLD" and state.get("online"):
            print("指令完成，等待下個指令", flush=True)
        self.previous_phase = phase

    def print_state(self):
        state = self.latest or {}
        fresh = self.last_rx is not None and time.monotonic() - self.last_rx < 2.0
        if not fresh or not state.get("online"):
            state = {}
        mode = state.get("mode")
        if not isinstance(mode, str) or not mode:
            mode = "UNKNOWN"
        position = state.get("position_ned")
        x = y = altitude = None
        if (isinstance(position, (list, tuple)) and len(position) == 3
                and all(type(v) in (int, float) and math.isfinite(v) for v in position)):
            x, y, down = position
            altitude = state.get("relative_altitude")
        print(f"PX4 mode: {mode}; armed: {state.get('armed', '--')}; phase: {state.get('phase', '--')}\n"
              f"position: x = {number(x)}, y = {number(y)}; altitude: {number(altitude)}\n"
              f"yaw = {number(state.get('yaw_deg'))}\n"
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
