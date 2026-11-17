"""Display live PX4 telemetry and plot measured and commanded N/E/Up positions."""

import json
import math
import time
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Monitor(Node):
    def __init__(self):
        super().__init__("px4_states_monitor")
        self.create_subscription(String, "/flight/state", self.state, 10)
        self.create_subscription(String, "/flight/event", self.event, 20)
        self.start = time.monotonic()
        self.next_print = 0.0
        self.last_rx = None
        self.previous_phase = None
        self.gap_reported = False
        self.rows = deque(maxlen=1200)
        self.fig, self.axes = plt.subplots(3, 1, sharex=True, figsize=(9, 7), constrained_layout=True)
        self.fig.suptitle("PX4 local position")
        self.curves = []
        for axis, label in zip(self.axes, ("X · North (m)", "Y · East (m)", "Z · Up (m)")):
            actual, = axis.plot([], [], label="Actual")
            target, = axis.plot([], [], "--", label="Offboard target")
            axis.set_ylabel(label)
            axis.grid(alpha=0.25)
            self.curves.append((actual, target))
        self.axes[0].legend(loc="upper left")
        self.axes[-1].set_xlabel("Elapsed time (s)")

    def event(self, msg):
        # HOLD's completion message is driven by state so late subscribers see it too.
        if msg.data != "指令完成，等待下個指令":
            print(msg.data, flush=True)

    def state(self, msg):
        s = json.loads(msg.data)
        now = time.monotonic()
        self.last_rx = now
        self.gap_reported = False
        pos = s["position_ned"] or [math.nan] * 3
        target = s["target_ned"] or [math.nan] * 3
        self.rows.append((now - self.start, pos[0], pos[1], -pos[2], target[0], target[1], -target[2]))
        phase = s["phase"]
        if phase == "HOLD":
            if self.previous_phase != "HOLD":
                print("指令完成，等待下個指令", flush=True)
        elif now >= self.next_print or self.previous_phase != phase:
            speed, yaw = s["speed"], s["yaw_deg"]
            speed_text = "?" if speed is None else f"{speed:.3f}"
            yaw_text = "?" if yaw is None else f"{yaw:.1f}"
            print(f"{phase} | PX4={s['mode']} armed={s['armed']} {s['landed']} | "
                  f"N/E/Up=({pos[0]:.2f}, {pos[1]:.2f}, {-pos[2]:.2f}) m | "
                  f"v={speed_text} m/s heading={yaw_text}°", flush=True)
            self.next_print = now + 1
        self.previous_phase = phase

    def draw(self):
        if self.last_rx is not None and time.monotonic() - self.last_rx > 2 and not self.gap_reported:
            print("控制節點狀態逾時", flush=True)
            self.gap_reported = True
            self.rows.append((time.monotonic() - self.start,) + (math.nan,) * 6)
        if not self.rows:
            return
        columns = list(zip(*self.rows))
        for i, (axis, curves) in enumerate(zip(self.axes, self.curves)):
            curves[0].set_data(columns[0], columns[i + 1])
            curves[1].set_data(columns[0], columns[i + 4])
            axis.relim()
            axis.autoscale_view()
        self.fig.canvas.draw_idle()


def main():
    rclpy.init()
    node = Monitor()
    plt.show(block=False)
    next_draw = 0.0
    try:
        while rclpy.ok() and plt.fignum_exists(node.fig.number):
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now >= next_draw:
                node.draw()
                next_draw = now + 0.2
            node.fig.canvas.flush_events()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        plt.close(node.fig)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
