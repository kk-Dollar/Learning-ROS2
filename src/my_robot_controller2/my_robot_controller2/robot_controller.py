#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial
class RobotController(Node):
    def __init__(self):
        self.prev_x=0
        super().__init__("robot_controller")
        self.cmd_vel_pub_=self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.pose_subscriber_=self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10)
    def pose_callback(self,pose:Pose):
        msg=Twist()
        if(pose.x>8.5 or pose.x<2.5 or pose.y>8.5 or pose.y<2.5):
            msg.linear.x=1.0
            msg.angular.z=0.9
        else:
            msg.linear.x=5.0
            msg.angular.z=0.0
        self.cmd_vel_pub_.publish(msg)     
        if pose.x>5.5 and self.prev_x<=5.5:
            self.set_pen_server(255,0,0,3,0)   
            self.get_logger().info("pen color switched to red")
            self.prev_x=pose.x
        elif pose.x<=5.5 and self.prev_x>5.5:
            self.set_pen_server(0,255,0,3,0)   
            self.get_logger().info("pen color switched to green")   
            self.prev_x=pose.x
    def set_pen_server(self,r,g,b,width,off):
        client= self.create_client(SetPen,"/turtle1/set_pen")        
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Service is not available")   
        request= SetPen.Request()
        request.r=r
        request.g=g
        request.b=b
        request.width=width
        request.off=off
        future= client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))
    def callback_set_pen(self,future):
        try:
            response=future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" %(e,))    
def main(args=None):
    rclpy.init(args=args)
    node=RobotController()
    rclpy.spin(node)
    rclpy.shutdown()