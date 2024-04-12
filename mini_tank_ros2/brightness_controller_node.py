import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from bittle_msgs.msg import Detection
from std_msgs.msg import Float64
import numpy as np

class BrightnessController(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            1
        )

        self.compass_subscription = self.create_subscription(
            Float64,
            'compass_degree',
            self.current_heading_callback,
            1
        )
        self.bridge = CvBridge()

        self.model = YOLO('/root/mobot.pt')
        self.model.conf = 0.5

        self.detection_publisher = DetectionPublisher()

        self.desired_heading = 0.0
        self.current_heading = 0.0

        self.straight_gain = 0.15
        self.center = 0
        self.camera_angle = 70
        self.img_size = (480, 640)
        self.cutoff_height = 160
        self.weights = np.ones([self.img_size[0] - self.cutoff_height, self.img_size[1]])
        for i in range(self.img_size[1]):
            self.weights[:,i] = (self.img_size[1]/2 - i) * self.camera_angle/self.img_size[1]

    def image_callback(self, data):
        try:
            self.get_logger().info('Receiving video frame')
            current_frame = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Perform object detection
            results = self.model([current_frame])

            # Check for detections and publish results
            if len(results) > 0:
                result_list = (results[0].boxes.cls).cpu().tolist()
                xywhn_list = (results[0].boxes.xywhn).cpu().tolist()
                detection_info = [{'results': result_list, 'xywhn_list': xywhn_list}]
                self.detection_publisher.publish_detection_info(detection_info)
            else:
                self.get_logger().info("No detections")
            self.desired_heading = self.get_goal_heading(current_frame, self.current_heading)
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {str(e)}')


    def current_heading_callback(self, msg):
        self.get_logger().info("Received a /compass_degree message!")
        self.current_heading = msg.data


    def get_goal_heading(self, frame, current_heading):
        brightness = self.white_activation(frame[self.cutoff_height:, :], dtype=np.float32)
        brightness -= np.mean(brightness)
        brightness = np.clip(brightness, 0, 255)
        brightness = brightness**2
        brightness = 255*brightness/np.max(brightness)

        angle = np.sum(brightness*self.weights)/np.sum(brightness)

        angle_pix = int((-angle/self.camera_angle)*320 + 320)

        out_frame = cv2.cvtColor(brightness.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        cv2.line(out_frame, (angle_pix, 0), (angle_pix, frame.shape[0]), (0, 255,0), 2)

        goal_heading = current_heading + angle - (current_heading - self.center)*self.straight_gain

        return goal_heading

    def white_activation(self, image, dtype=np.uint8):
        # need to detect white pain, so we need to convert the image to grayscale, and penalize non-white pixels
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.int64)
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB).astype(np.int64)
        # penalize non-white pixels
        color_cost = abs(lab[:, :, 1] - 128) + abs(lab[:, :, 2] - 128)
        # penalize non-white pixels

        return (gray - color_cost.astype(np.int64)).clip(0, 255).astype(dtype)
    
    def threshold_image(self,image):
        white_activated_image = self.white_activation(image)
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        white_activated_image = cv2.GaussianBlur(white_activated_image, (21, 21), 0)

        # use otzu thresholding
        otzu_value, _ = cv2.threshold(white_activated_image[white_activated_image > 100].flatten(), 0, 255, cv2.THRESH_OTSU)

        threshold = 255 - 0.85*(255-otzu_value)

        _, thresholded_image = cv2.threshold(white_activated_image, threshold, 255, cv2.THRESH_BINARY)

        return thresholded_image
        
class DetectionPublisher(Node):
    def __init__(self):
        super().__init__('detection_publisher')
        self.publisher = self.create_publisher(Detection, 'detection_topic', 10)
        
    def publish_detection_info(self, detection):
        msg = Detection()
        msg.results = [int(result) for result in detection[0]['results']]
        print('results:', msg.results)
        # Initialize an empty list to hold the flattened and converted values
        flattened_xywhn_list = []

        # Iterate through each sublist in 'xywhn_list'
        for sublist in detection[0]['xywhn_list']:
            # Check if the item is a list (to handle nested lists)
            if isinstance(sublist, list):
                # Iterate through each item in the sublist
                for item in sublist:
                    # Convert each item to float and append to the flattened list
                    flattened_xywhn_list.append(float(item))
            else:
                # If the item is not a list, convert and append directly
                flattened_xywhn_list.append(float(sublist))

        # Assign the flattened list of floats to 'msg.xywhn_list'
        msg.xywhn_list = flattened_xywhn_list
        print('xywhn_list:', msg.xywhn_list)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing detection')


def main(args=None):
    rclpy.init(args=args)
    brightness_controller = BrightnessController()
    rclpy.spin(brightness_controller)
    brightness_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



