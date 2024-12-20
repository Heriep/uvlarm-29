#!python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32
import math
import time

rosNode= None
THRESHOLD_DIST = 0.4

def infiniteTalk():
    # Initialize ROS node with ROS client
    rclpy.init()
    aNode = Node('basic_move')
    control = StraightCtrl(aNode)
    # Start infinite loop
    rclpy.spin(aNode)
    # Clean everything and switch the light off
    aNode.destroy_node()
    rclpy.shutdown()

class StraightCtrl:
    def __init__(self, rosNode):
        self.rosNode = rosNode
        self.rosNode.create_subscription( LaserScan, 'scan', self.scan_callback, 10)
        self._publisher= rosNode.create_publisher(Twist, '/multi/cmd_nav', 10 )
        self._timer = rosNode.create_timer(0.1, self.control_callback)
        self._i = 0
        self.dist_min = float('inf')
        self.angle_dist_min = 0

    def control_callback(self):
        velocity = Twist()
        # Feed Twist velocity values
        velocity.linear.x = 0.15
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0

        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = 0.0

        if self.dist_min <= THRESHOLD_DIST :
            if self.angle_dist_min <= 0 :
                velocity.linear.x = 0.0
                velocity.angular.z = 1.0
            else :
                velocity.linear.x = 0.0
                velocity.angular.z = -1.0

        # Publish 
        self._publisher.publish(velocity)

    def scan_callback(self, scanMsg):    
        self.dist_min = float('inf')
        angle= scanMsg.angle_min
        for aDistance in scanMsg.ranges :
            if 0.2 < aDistance and aDistance < 5.0 and (scanMsg.angle_min + 0.8 <= angle <= scanMsg.angle_max - 0.8 ) :
                if self.dist_min >= aDistance:
                    self.dist_min = aDistance
                    self.angle_dist_min = angle

            angle+= scanMsg.angle_increment

        print(scanMsg.angle_max)


# Execute the function.
if __name__ == "__main__":
    infiniteTalk()
