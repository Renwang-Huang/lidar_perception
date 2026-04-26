#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class HumbleOdomRelay(Node):
    def __init__(self):
        super().__init__('humble_odom_relay_node')
        
        self.publisher_ = self.create_publisher(
            PoseStamped, 
            '/mavros/vision_pose/pose', 
            10
        )
        
        self.subscription = self.create_subscription(
            Odometry,
            '/lidar_odom/Odometry',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):

        target_struct = PoseStamped()
        
        target_struct.header.stamp = msg.header.stamp
        target_struct.header.frame_id = msg.header.frame_id
        
        target_struct.pose.position.x = msg.pose.pose.position.x
        target_struct.pose.position.y = msg.pose.pose.position.y
        target_struct.pose.position.z = msg.pose.pose.position.z

        ox = msg.pose.pose.orientation.x
        oy = msg.pose.pose.orientation.y
        oz = msg.pose.pose.orientation.z
        ow = msg.pose.pose.orientation.w
        
        target_struct.pose.orientation.x = ox * 0.7071 - ow * 0.7071
        target_struct.pose.orientation.y = oy * 0.7071 - oz * 0.7071
        target_struct.pose.orientation.z = oz * 0.7071 + oy * 0.7071
        target_struct.pose.orientation.w = ow * 0.7071 + ox * 0.7071

        self.publisher_.publish(target_struct)

def main(args=None):
    rclpy.init(args=args)
    
    node = HumbleOdomRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C...')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()