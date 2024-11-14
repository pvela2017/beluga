#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # Import Odometry message type
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class OdomRepublisher(Node):

    def __init__(self):
        super().__init__('odom_republisher')

        # Subscribe to the input Odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',  # Change this to your odometry topic name
            self.odom_callback,
            10)

        # Publisher for the republished PoseStamped with a different frame_id
        self.publisher_ = self.create_publisher(PoseStamped, 'gt_poses_map', 10)

        # Create a TransformBroadcaster to publish tf
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):
        # Modify the frame_id
        new_frame_id = 'odom'

        # Create a new PoseStamped message from the Odometry message
        new_msg = PoseStamped()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = new_frame_id
        new_msg.pose = msg.pose.pose  # Extract the pose from the Odometry message

        # Publish the modified PoseStamped
        self.publisher_.publish(new_msg)

        # Create and publish the TransformStamped based on the PoseStamped
        transform = TransformStamped()
        transform.header.stamp = new_msg.header.stamp
        transform.header.frame_id = new_msg.header.frame_id
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = new_msg.pose.position.x
        transform.transform.translation.y = new_msg.pose.position.y
        transform.transform.translation.z = new_msg.pose.position.z
        transform.transform.rotation = new_msg.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdomRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
