import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# OpenCV 관련
from cv_bridge import CvBridge
import cv2


# Subscriber 노드 생성
class RealSenseRGBSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_rgb_subscriber') # 노드 이름

        self.bridge = CvBridge() # 이번엔 ROS Image -> OpenCV 로 바꾸려고

        self.subscription = self.create_subscription(
            Image,
            '/realsense/color/image_raw',   # Publisher와 동일
            self.callback,
            10
        )

        self.get_logger().info(
            'Subscribed to /realsense/color/image_raw'
        )

# 구독시 할 일
    def callback(self, msg):
        # ROS Image → OpenCV 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 영상 화면 띄우기
        cv2.imshow('RealSense RGB (subscriber)', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseRGBSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
