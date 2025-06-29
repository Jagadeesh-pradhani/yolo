import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from yolo.easy_yolov7.algorithm.object_detector import YOLOv7
from yolo.easy_yolov7.utils.detections import draw
import cv2
import os
import math
from ament_index_python import get_package_share_directory
import numpy as np

class YOLOv7MultiDrone(Node):
    def __init__(self):
        super().__init__('yolov7_multi_drone')

        # Declare / read number of drones
        self.declare_parameter('num_drones', 1)
        self.num_drones = self.get_parameter('num_drones').value

        # YOLOv7 setup
        package_dir = get_package_share_directory('yolo')
        weights = os.path.join(package_dir, 'yolo', 'easy_yolov7', 'coco.weights')
        classes = os.path.join(package_dir, 'yolo', 'easy_yolov7', 'coco.yaml')
        self.yolov7 = YOLOv7()
        self.yolov7.load(weights, classes=classes, device='gpu')

        self.bridge = CvBridge()

        # Holders for publishers
        self.coord_pubs = []
        self.image_pubs = []

        # Buffer for latest frames
        self.latest_frames = [None] * self.num_drones

        # Create subs & pubs
        for i in range(self.num_drones):
            # raw image sub
            self.create_subscription(
                Image,
                f'/camera_sensor_{i}/image_raw',
                callback=self._make_callback(i),
                qos_profile=10
            )

            # centroid publisher
            self.coord_pubs.append(
                self.create_publisher(Point, f'/drone_{i}/xy_coordinates', 10)
            )

            # (optional) individual image publisher
            self.image_pubs.append(
                self.create_publisher(Image, f'/camera_sensor_{i}/image_detect', 10)
            )

        # Open a named window for the mosaic
        cv2.namedWindow("All Detections", cv2.WINDOW_NORMAL)
        # you can fix size or let it autoscale
        self.get_logger().info(f'Initialized for {self.num_drones} drones.')

    def _make_callback(self, idx):
        def callback(msg):
            # convert
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                self.get_logger().error(f'[{idx}] CvBridge error: {e}')
                return

            # detect & draw
            detections = self.yolov7.detect(frame)
            drawn = draw(frame, detections)

            # compute centroids & publish
            for det in detections:
                if det['class'] == 'bullseye':  # When bullseye is detected
                    x, y, w, h = det['x'], det['y'], det['width'], det['height']
                    cx, cy = x + w/2, y + h/2
                    cv2.circle(drawn, (int(cx), int(cy)), 5, (0,255,0), -1)
                    cv2.putText(drawn, "BULLSEYE", (int(x), int(y-10)), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    self._publish_point(idx, cx, cy)
                    self.get_logger().info(f"Drone {idx} detected bullseye at pixel coordinates: ({cx}, {cy})")
                    continue
                x, y, w, h = det['x'], det['y'], det['width'], det['height']
                cx, cy = x + w/2, y + h/2
                cv2.circle(drawn, (int(cx), int(cy)), 5, (0,0,255), -1)
                self._publish_point(idx, cx, cy)

            # publish individual image (optional)
            self._publish_image(idx, drawn)

            # update buffer & redraw mosaic
            self.latest_frames[idx] = drawn
            self._show_mosaic()

        return callback

    def _publish_point(self, idx, x, y):
        self.get_logger().info(f'[{idx}] Publishing point: ({x}, {y})')
        msg = Point(x=x, y=y, z=0.0)
        self.coord_pubs[idx].publish(msg)

    def _publish_image(self, idx, frame):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pubs[idx].publish(img_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'[{idx}] CvBridge error: {e}')

    def _show_mosaic(self):
        # determine grid size (roughly square)
        n = self.num_drones
        cols = math.ceil(math.sqrt(n))
        rows = math.ceil(n / cols)

        # first, decide cell size: use smallest frame size among available
        heights, widths = [], []
        for f in self.latest_frames:
            if f is not None:
                h, w = f.shape[:2]
                heights.append(h)
                widths.append(w)
        if not heights:
            return  # no frames yet

        cell_h = min(heights)
        cell_w = min(widths)

        # build rows of images
        grid_rows = []
        idx = 0
        for r in range(rows):
            row_imgs = []
            for c in range(cols):
                if idx < n and self.latest_frames[idx] is not None:
                    # resize to cell
                    img = cv2.resize(self.latest_frames[idx], (cell_w, cell_h))
                else:
                    # black image placeholder
                    img = 255 * np.zeros((cell_h, cell_w, 3), dtype=np.uint8)
                row_imgs.append(img)
                idx += 1
            # horizontal concat
            grid_rows.append(cv2.hconcat(row_imgs))

        # vertical concat all rows
        mosaic = cv2.vconcat(grid_rows)
        cv2.imshow("All Detections", mosaic)
        cv2.waitKey(1)

    def __del__(self):
        self.yolov7.unload()
        cv2.destroyWindow("All Detections")


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv7MultiDrone()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
