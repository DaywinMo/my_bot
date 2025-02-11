#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class camera_sub(Node):

    def __init__(self):
        super().__init__('qr_maze_solving_node')
        self.camera_sub = self.create_subscription(Image,'/camera/image_raw',self.camera_cb,10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.vel_msg=Twist()
        self.bridge=CvBridge()



    def camera_cb(self, data):
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        frame = frame[290:479,137:517] # [Y1:Y2, X1,X2]
        edged = cv2.Canny(frame ,60,100 )

        white_index=[]
        mid_point_line = 0
        for index,values in enumerate(edged[:][172]):
            if(values == 255):
                white_index.append(index)

        white_index = white_index[::len(white_index)-1]
        print(white_index)

        max_lin_speed = 0.4
        if(len(white_index) == 2 ):
            cv2.circle(img=edged, center = (white_index[0],172), radius = 2 , color = (255,0,0), thickness=1)
            cv2.circle(img=edged, center = (white_index[1],172), radius = 2 , color = (255,0,0), thickness=1)
            mid_point_line = int ( (white_index[0] + white_index[1]) /2 )
            cv2.circle(img=edged, center = (mid_point_line,172), radius = 3 , color = (255,0,0), thickness=2)

            mid_point_robot = [190,188]
            cv2.circle(img=edged, center = (mid_point_robot[0],mid_point_robot[1]), radius = 5 , color = (255,0,0), thickness=2)
            error = mid_point_robot[0] - mid_point_line
            print("Error -> " , error)

            # Factor creation
            #error * factor
            # Z scale 0.01 to 0.5
            abs_error = abs(error)
            ang_speed = error*0.025
            
            lin_speed = 0.0

            if (abs_error > 20):
                lin_speed = max_lin_speed*0.9
            else:
                lin_speed = max_lin_speed

            print(f"{lin_speed} / {ang_speed}")

            if ( error != 0):
                self.vel_msg.angular.z = ang_speed

            else:
                self.vel_msg.angular.z = 0.0

            self.vel_msg.linear.x = lin_speed

        else:
            self.vel_msg.angular.z = 0.0
            self.vel_msg.linear.x = max_lin_speed

        
        self.cmd_vel_pub.publish(self.vel_msg)


        cv2.imshow('Frame',frame)
        cv2.imshow('Canny Output',edged)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    sensor_sub = camera_sub()

    rclpy.spin(sensor_sub)
    sensor_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()