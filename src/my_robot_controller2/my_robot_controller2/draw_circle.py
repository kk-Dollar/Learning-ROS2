#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

class DrawCircle(Node):
    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub=self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.timer=self.create_timer(0.001,self.change_vel)
        self.get_logger().info("Draw Circle node started.....")
    def change_vel(self):
        msg=Twist()
        msg.linear.x=1.0
        msg.angular.z=2.0
        self.cmd_vel_pub.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node=DrawCircle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='main':
    main()    