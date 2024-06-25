#!/usr/bin/env python3

# ROS imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String  # Added for String message type
import cv2

# Barcode scanning imports
from pyzbar.pyzbar import decode
import os
import subprocess

# Custom function to play sound and suppress output
def play_sound(file_path):
    FNULL = open(os.devnull, 'w')
    subprocess.call(['ffplay', '-nodisp', '-autoexit', file_path], stdout=FNULL, stderr=subprocess.STDOUT)

class BarcodeReader(Node):
    def __init__(self):
        super().__init__('barcode_reader')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Input image topic
            self.image_callback,
            10)
        self.barcode_image_pub = self.create_publisher(Image, 'barcode_imageviewer', 10)  # New topic for barcode-detected frames
        self.encode_barcode_pub = self.create_publisher(String, 'encoded_barcode', 10)  # Publisher for encoded barcode data
        self.last_scanned_barcode = None  # Keep track of the last scanned barcode

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            barcode_img, barcode_data, top_left, bottom_right = self.detect_barcode(cv_image)
            if barcode_data is not None and barcode_data != self.last_scanned_barcode:  # Check if the barcode is new
                play_sound("/home/arief/FYPworkspaces/fyp_ros2_ws/src/node_redfyp/sounds/beep-07a.wav")  # Play the new beep sound
                play_sound("/home/arief/FYPworkspaces/fyp_ros2_ws/src/node_redfyp/sounds/beep-07a.wav")  # Play the beep sound again
                self.last_scanned_barcode = barcode_data  # Update the last scanned barcode
                # Publish the encoded barcode data
                encoded_barcode_msg = String()
                encoded_barcode_msg.data = self.encode_barcode(barcode_data)
                self.encode_barcode_pub.publish(encoded_barcode_msg)
                print("New barcode detected and encoded data published!")
            # Draw a green rectangle around the detected barcode
            if top_left and bottom_right:
                cv2.rectangle(barcode_img, top_left, bottom_right, (0, 255, 0), 2)
                # Display barcode data as text on the image
                text = "Barcode Value: {}".format(barcode_data)
                cv2.putText(barcode_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
            # Publish the barcode-detected frame
            barcode_image_msg = self.bridge.cv2_to_imgmsg(barcode_img, encoding='bgr8')
            self.barcode_image_pub.publish(barcode_image_msg)
        except Exception as e:
            self.get_logger().error("Error processing image: {}".format(str(e)))

    def detect_barcode(self, frame):
        detected_barcode = decode(frame)
        if detected_barcode:
            barcode = detected_barcode[0]  # Only consider the first detected barcode
            if barcode.data:
                # Return the frame, decoded data, and barcode's location
                return frame, barcode.data.decode('utf-8'), (barcode.rect.left, barcode.rect.top), (barcode.rect.left + barcode.rect.width, barcode.rect.top + barcode.rect.height)
        # Return the original frame and None for the other values when no barcode is detected
        return frame, None, None, None

    def encode_barcode(self, barcode_data):
        if barcode_data is not None:
            # Your encoding logic here, for example, you can just return the data itself
            return barcode_data
        else:
            return None  # or return an empty string, depending on what you expect downstream

def main(args=None):
    rclpy.init(args=args)
    barcode_reader = BarcodeReader()
    print("Barcode Reader is Running...")
    try:
        rclpy.spin(barcode_reader)
    except KeyboardInterrupt:
        barcode_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
