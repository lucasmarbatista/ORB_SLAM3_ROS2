#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import cv2
import os
import sys
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImagePublisher(Node):
    def __init__(self, file_path):
        super().__init__("image_publisher")
        self.bridge = CvBridge()
        # Fill it with the video complete path, without ~ (tilde sign)
        # Example: self.cap = cv2.VideoCapture("/home/lucasmarins/test_slam_ws/src/ORB_SLAM3_ROS2/videos/example_video.avi")
        self.cap = cv2.VideoCapture("/home/user/slam_ws/src/ORB_SLAM3_ROS2/videos/example_video.avi")
        self.pub = self.create_publisher(Image, "/camera", 10)

    def run(self):
        print(self.cap.isOpened())
        print(cv2.getBuildInformation())
        last_publish_time = self.get_clock().now()
        while(True):
            ret, frame = self.cap.read() 
            if ret:
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            time.sleep(0.01) # Set to run the video on the same rate that was recorded
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    # if(len(sys.argv) is not 2):
    #     print("Incorrect number of arguments\nUsage:\n\tpython3 <path_to_video_file>")
    #     exit()

    if not os.path.isfile(sys.argv[0]):
        print("Invalid file path")
        exit()

    ip = ImagePublisher(sys.argv[0])
    print("Publishing...")
    ip.run()
    ip.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()