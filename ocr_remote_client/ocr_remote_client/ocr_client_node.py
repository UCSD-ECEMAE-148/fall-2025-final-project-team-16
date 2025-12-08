#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import socket
import struct
import cv2


class OcrClientNode(Node):
    def __init__(self):
        super().__init__('ocr_client_node')

        # 参数：Mac 上 OCR server 的 IP & 端口
        # 记得改成你自己的 Mac 局域网 IP，比如 192.168.1.23
        self.declare_parameter('server_host', '192.168.10.133')
        self.declare_parameter('server_port', 5001)

        self.server_host = self.get_parameter('server_host').get_parameter_value().string_value
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value

        # 订阅相机图像
        # 相机 topic 参数，默认值先给一个常见的
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'Subscribing to image topic: {image_topic}')


        # 发布 OCR 文字
        self.text_pub = self.create_publisher(String, '/ocr_text', 10)

        self.bridge = CvBridge()
        self.frame_count = 0  # 控制频率用，避免每帧都发

        self.get_logger().info(
            f'OCR client node initialized. Server: {self.server_host}:{self.server_port}'
        )

    # 简单的 socket 收包函数
    def _recv_all(self, sock, length):
        data = b""
        while len(data) < length:
            try:
                packet = sock.recv(length - len(data))
            except socket.timeout:
                return None
            if not packet:
                return None
            data += packet
        return data

    def send_image_and_get_text(self, img):
        """
        img: OpenCV BGR image
        return: recognized text (str) or "" on error
        """
        # 压缩为 JPEG，减少网络传输量
        ok, encoded = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            self.get_logger().error("Failed to encode image")
            return ""

        data = encoded.tobytes()
        length = len(data)

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(1.5)  # 适当设置超时
                s.connect((self.server_host, self.server_port))

                # 发送长度 + 数据
                s.sendall(struct.pack("!I", length))
                s.sendall(data)

                # 接收返回的文本长度
                header = self._recv_all(s, 4)
                if not header:
                    self.get_logger().warn("No response header from server")
                    return ""
                (txt_len,) = struct.unpack("!I", header)

                # 接收文本内容
                txt_bytes = self._recv_all(s, txt_len)
                if not txt_bytes:
                    self.get_logger().warn("No text payload from server")
                    return ""

                return txt_bytes.decode("utf-8", errors="ignore")

        except Exception as e:
            self.get_logger().warn(f"Failed to contact OCR server: {e}")
            return ""

    def image_callback(self, msg: Image):
        # 控制频率：比如每 5 帧处理一次
        self.frame_count += 1
        if self.frame_count % 5 != 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        text = self.send_image_and_get_text(cv_image)
        if text:
            out = String()
            out.data = text
            self.text_pub.publish(out)
            self.get_logger().info(f"OCR text: {text}")
        # 如果没有 text，就安静地跳过即可


def main(args=None):
    rclpy.init(args=args)
    node = OcrClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
