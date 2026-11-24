"""Keyboard input: absolute XY coordinates relative to the fixed takeoff target."""

import json
import math
import uuid

def read_request(read=input, display=print):
    choice = read("[0 解鎖 / 1 起飛 / 2 追蹤 / 3 降落 / kill 緊急停轉 / q 離開] > ").strip().lower()
    mode = {"0": "arm", "1": "takeoff", "2": "offboard", "3": "land", "q": "quit"}.get(choice, choice)
    if mode in {"arm", "land", "kill", "quit"}:
        return {"mode": mode}
    if mode == "takeoff":
        while True:
            value = read("起飛高度 m（相對本次起點，大於 0；Enter=controller 預設；cancel / land / kill）> ").strip().lower()
            if value == "cancel":
                return None
            if value in {"land", "kill"}:
                return {"mode": value}
            if value == "":
                return {"mode": "takeoff"}
            try:
                height = float(value)
                if not math.isfinite(height) or height <= 0:
                    raise ValueError
                return {"mode": "takeoff", "height": height}
            except ValueError:
                display("高度請輸入大於 0 的有限數值，單位公尺。")
    if mode != "offboard":
        display("請輸入 0、1、2、3、kill 或 q。")
        return None
    request = {"mode": mode}
    display("XY 原點＝本次起飛目標點；Z 維持起飛高度；座標不累加。")
    for axis, direction in (("x", "北"), ("y", "東")):
        while True:
            value = read(f"{axis} = 相對起飛目標原點的座標 m（正向{direction}，Enter=0，cancel 取消，land 降落，kill 停轉）> ").strip().lower()
            if value == "cancel":
                return None
            if value in {"land", "kill"}:
                return {"mode": value}
            try:
                number = float(value or 0)
                if not math.isfinite(number):
                    raise ValueError
                request[axis] = number
                break
            except ValueError:
                display("請輸入有限數值。")
    return request


def main():
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from std_msgs.msg import String

    rclpy.init()
    node = Node("mode_select")
    pub = node.create_publisher(String, "/flight/request",
                                QoSProfile(depth=10, lifespan=Duration(seconds=2)))
    try:
        while rclpy.ok():
            request = read_request()
            if request is None:
                continue
            if request["mode"] == "quit":
                break
            if pub.get_subscription_count() == 0:
                print("控制節點尚未連線。此次未送出，請確認 main.py 已啟動。")
                continue
            request["id"] = uuid.uuid4().hex
            pub.publish(String(data=json.dumps(request, allow_nan=False)))
            print(f"已送出 {request}。是否接受請看 controller；q 只關閉鍵盤，不會降落。")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
