# # 1. object detection check - visualization
# # 2. coordination - camera to object x -> baselink to object TF x (calibration -> T_,,,,.npy)
# # 3. 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import time


class ImgNode(Node):
    def __init__(self):
        super().__init__('img_node')
        self.bridge = CvBridge()
        self.color_frame = None
        self.intrinsics = None

        self.color_subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10)

        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

    def color_callback(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def camera_info_callback(self, msg):
        self.intrinsics = {
            "fx": msg.k[0], "fy": msg.k[4],
            "ppx": msg.k[2], "ppy": msg.k[5]
        }

    def get_color_frame(self):
        return self.color_frame


class YoloTargetFilter(Node):
    def __init__(self):
        super().__init__('yolo_target_filter')

        self.target_sub = self.create_subscription(String, '/detected_tool', self.target_callback, 10)
        self.target_keyword = None
        self.publisher_ = self.create_publisher(String, 'yolo_result', 10)

        # YOLO 모델 불러오기
        self.model = YOLO("/home/rokey/Downloads/gasStation_1.pt")
        self.names = self.model.names

        # 내부에서 ImgNode 사용
        self.img_node = ImgNode()

        self.get_logger().info("YOLO 추론 노드 시작 (RealSense 이미지 기반)")

    def _get_depth(self, x, y):
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return float(frame[y, x])
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )

    def target_callback(self, msg):
        self.target_keyword = msg.data.lower()
        self.get_logger().info(f"[STT] 타겟 키워드 업데이트: {self.target_keyword}")

    def publish_detection(self, class_name, cx_cal, cy_cal, cz_cal, conf):
        msg = String()
        msg.data = f"{class_name} {conf:.2f} {cx_cal} {cy_cal} {cz_cal}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self.img_node, timeout_sec=0.1)
            frame = self.img_node.get_color_frame()

            if frame is None or self.target_keyword is None:
                continue

            results = self.model.predict(source=frame, conf=0.4, verbose=False, visualize=True)[0]
            
            if results.boxes is None or len(results.boxes) == 0:
                    
                self.get_logger().warn(f"[YOLO] target '{self.target_keyword}' failed detect")
                continue

            for box in results.boxes:
                cx, cy, w, h = map(int, box.xywh[0])
                cz = self._get_depth(cx, cy)
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = self.names[cls_id].lower()

                if self.intrinsics is None or cz is None:
                    self.get_logger().warn("intrinsics 또는 depth 없음. 건너뜀.")
                    continue

                cx_cal, cy_cal, cz_cal = self._pixel_to_camera_coords(cx, cy, cz)

                if cls_name == self.target_keyword:
                    self.publish_detection(cls_name, cx_cal, cy_cal, cz_cal, conf)


            time.sleep(0.1)


def main():
    rclpy.init()
    node = YoloTargetFilter()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()