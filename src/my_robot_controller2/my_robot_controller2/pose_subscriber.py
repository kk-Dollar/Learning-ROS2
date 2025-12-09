#!/use/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubcriber(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber= self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10)
    def pose_callback(self,msg:Pose):
        self.get_logger().info("x: "+ str(msg.x) + "y: "+ str(msg.y))
def main(args=None):
    rclpy.init(args=args)
    node = PoseSubcriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='main':
    main()    