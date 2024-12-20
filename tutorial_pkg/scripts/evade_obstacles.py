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
    aNode = Node('scan_interpreter')
    talker = ROSNav(aNode)
    # Start infinite loop
    rclpy.spin(aNode)
    # Clean everything and switch the light off
    aNode.destroy_node()
    rclpy.shutdown()

class ROSNav:
    def __init__(self, rosNode):
        self.rosNode = rosNode
        self.rosNode.create_subscription( LaserScan, 'scan', self.scan_callback, 10)
        self._publisher= rosNode.create_publisher(Twist, '/multi/cmd_nav', 10 )
        self._timer = rosNode.create_timer(0.1, self.timer_callback)
        self._i = 0
        self.dist_min = float('inf')
        self.angle_dist_min = 0

    def timer_callback(self):
        velocity = Twist()
        # Feed Twist velocity values
        velocity.linear.x = 0.2
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


'''def scan_callback(scanMsg):
    global rosNode

    obstacles= []
    angle= scanMsg.angle_min
    for aDistance in scanMsg.ranges :
        if 0.1 < aDistance and aDistance < 5.0 :
            aPoint= Point32()
            aPoint.x= (float)(math.cos(angle) * aDistance)
            aPoint.y= (float)(math.sin( angle ) * aDistance)
            aPoint.z= (float)(0)
            obstacles.append(aPoint)

        angle+= scanMsg.angle_increment

    output = PointCloud()
    output.points = obstacles
    output.header.frame_id = 'base_link'
    aPublisher.publish(output)

rclpy.init()
rosNode= Node('scan_interpreter')
rosNode.create_subscription( LaserScan, 'scan', scan_callback, 10)
aPublisher= rosNode.create_publisher(PointCloud, 'sensor_msgs/PointCloud2', 10 )

while True :
    rclpy.spin_once( rosNode )
scanInterpret.destroy_node()
rclpy.shutdown()'''