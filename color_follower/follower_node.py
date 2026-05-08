import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorFollower(Node):
    def __init__(self):
        super().__init__('color_follower')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.Kp_angular = 0.002
        self.angular_threshold = 30
        self.min_lidar_distance = 0.30
        self.closest_obstacle = float('inf')
        self.get_logger().info('Color Follower Node Started!')

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        front = np.concatenate([ranges[0:30], ranges[330:360]])
        front = front[np.isfinite(front)]
        if len(front) > 0:
            self.closest_obstacle = float(np.min(front))

    def image_callback(self, msg):
        if self.closest_obstacle < self.min_lidar_distance:
            self.get_logger().info(f'LiDAR STOP — closest: {self.closest_obstacle:.2f}m')
            self.publisher.publish(Twist())
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        image_center_x = w // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([59, 240, 95])
        upper_green = np.array([61, 255, 110])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Apply ROI — center 50% width, lower 70% height
        mask_roi = np.zeros_like(mask)
        roi_start = w // 4
        roi_end = 3 * w // 4
        mask_roi[:, roi_start:roi_end] = mask[:, roi_start:roi_end]
        mask_roi[:int(h * 0.3), :] = 0

        contours, _ = cv2.findContours(
            mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        sphere_found = False

        if contours:
            candidates = []
            for c in contours:
                area = cv2.contourArea(c)
                if area < 200:
                    continue
                M = cv2.moments(c)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                candidates.append((cy, cx, area, c))

            if candidates:
                candidates.sort(reverse=True)
                cy, cx, area, best = candidates[0]
                error = image_center_x - cx
                self.get_logger().info(
                    f'SPHERE cx:{cx} cy:{cy} error:{error} area:{area:.0f} '
                    f'lidar:{self.closest_obstacle:.2f}m')
                sphere_found = True
                if abs(error) < self.angular_threshold:
                    twist.linear.x = 0.15
                    twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = self.Kp_angular * error

        if not sphere_found:
            self.get_logger().info('No sphere — spinning to recover...')
            twist.angular.z = 0.08

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ColorFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()