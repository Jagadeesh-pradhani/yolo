import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from yolo.easy_yolov7.algorithm.object_detector import YOLOv7
from yolo.easy_yolov7.utils.detections import draw
import cv2
import json
import sys
import os
import imutils
from ament_index_python import get_package_share_directory

class YOLOv7Detection(Node):
    def __init__(self):
        super().__init__('Detection')

        # Get package share directory
        package_dir = get_package_share_directory('yolo')
        
        # Construct paths to model files
        weights_path = os.path.join(package_dir, 'yolo', 'easy_yolov7', 'coco.weights')
        yaml_path = os.path.join(package_dir, 'yolo', 'easy_yolov7', 'coco.yaml')

        self.subscription = self.create_subscription(
            Image,
            '/camera_sensor_0/image_raw',
            self.listener_callback,
            10)
        self.subscription

        self.publisher_ = self.create_publisher(Point, 'xy_coordiantes', 10)

        self.cam_publisher_ = self.create_publisher(Image, '/camera_sensor_0/image_detect', 10)


        self.bridge = CvBridge()
        self.yolov7 = YOLOv7()
        self.yolov7.load(weights_path, classes=yaml_path, device='gpu')  # use 'gpu' for CUDA GPU inference
    
    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting ROS Image message to OpenCV image: {e}')
            return
        #frame = cv2.resize(frame, (640, 480))
        detections = self.yolov7.detect(frame)
        custom_detections = [detection for detection in detections if detection['class'] != 'person']
        detected_frame = draw(frame, detections)
        for detection in custom_detections:
            x, y, w, h = detection['x'], detection['y'], detection['width'], detection['height']
            centroid_x = x + (w / 2) # calculate centroid x-coordinate
            centroid_y = y + (h / 2) # calculate centroid y-coordinate
            cv2.circle(detected_frame, (int(centroid_x), int(centroid_y)), 5, (0, 0, 255), -1)  # draw centroid as a red circle

            print("x = ",centroid_x, "y = ",centroid_y)
            self.publish_tuple(centroid_x, centroid_y)

        #resized_frame = cv2.resize(detected_frame, (640, 480))
        self.publish_image(detected_frame)
        
        # cv2.imshow('YOLOv7 Detection', detected_frame)
        # cv2.waitKey(1)

    def publish_tuple(self, x, y):
        msg = Point()
        msg.x = x
        msg.y = y
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published: ({msg.x}, {msg.y})')
    
    def publish_image(self, frame):
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.cam_publisher_.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting OpenCV image to ROS Image message: {e}')
            return

    def __del__(self):
        self.yolov7.unload()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv7Detection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
