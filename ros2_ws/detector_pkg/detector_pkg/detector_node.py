import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.callback, 10)
        self.pub = self.create_publisher(Image, '/detections/image', 10)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, verbose=False)

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            class_name = self.model.names[int(box.cls)]
            confidence = float(box.conf)

            # Bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # Etiqueta
            label = f'{class_name} {confidence:.2f}'
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            self.get_logger().info(f'Detectado: {label}')

        # Publicar imagen anotada
        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

def main():
    rclpy.init()
    rclpy.spin(DetectorNode())
