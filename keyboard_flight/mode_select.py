"""Interactive keyboard node. Enter a whole relative NEU vector before publishing."""

import json
import math
import uuid

def read_request(read=input, display=print):
    mode = read("模式 [takeoff / offboard / land / quit] > ").strip().lower()
    if mode in {"takeoff", "land", "quit"}:
        return {"mode": mode}
    if mode != "offboard":
        display("請輸入 takeoff、offboard 或 land。")
        return None
    request = {"mode": mode}
    for axis, direction in (("x", "北"), ("y", "東"), ("z", "上")):
        while True:
            value = read(f"{axis} = 位移公尺（正向{direction}，Enter=0，cancel 取消）> ").strip().lower()
            if value == "cancel":
                return None
            if value == "land":
                return {"mode": "land"}
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
            print(f"已送出 {request['mode']}。是否接受請看監控端。")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
