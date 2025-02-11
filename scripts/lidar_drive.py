#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2

class lidar_camera_sub(Node):

    def __init__(self):
        super().__init__('qr_maze_solving_node')
        self.camera_sub = self.create_subscription(Image,'/camera/image_raw',self.camera_cb,10)
        self.lidar_sub = self.create_subscription(LaserScan,'/scan',self.lidar_cb,10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

        self.bridge=CvBridge()
        self.frame=0
        self.vel_msg = Twist()
        self.safe_distance = 0.5


    def camera_cb(self, data):
        self.frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        cv2.imshow('Frame',self.frame)
        cv2.waitKey(1)

    def lidar_cb(self, data):

        ranges_left = data.ranges[150:160]
        ranges_right = data.ranges[200:210]
        ranges = data.ranges[150:210]
        #print(ranges)
        min_distance_left = min(ranges_left)
        min_distance_right = min(ranges_right)
        if min_distance_left < self.safe_distance:
            #self.qr_detector()
            print("Object detected")
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.2

        elif min_distance_right < self.safe_distance:
            #self.qr_detector()
            print("Object detected")
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = -0.2

        else:
            print("No detect")
            self.vel_msg.linear.x = 0.2
            self.vel_msg.angular.z = 0.0
            
        self.cmd_vel_pub.publish(self.vel_msg)

    def qr_detector(self):
        decoder = cv2.QRCodeDetector()
        data, points, _ = decoder.detectAndDecode(self.frame)
        print(data)

def main(args=None):
    rclpy.init(args=args)

    sensor_sub = lidar_camera_sub()

    rclpy.spin(sensor_sub)
    sensor_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()