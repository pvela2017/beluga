#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class PoseRepublisher(Node):

    def __init__(self):
        super().__init__('pose_republisher')

        # Subscribe to the input PoseStamped topic
        self.subscription = self.create_subscription(
            PoseStamped,
            'gt_poses',
            self.pose_callback,
            10)

        # Publisher for the republished PoseStamped with a different frame_id
        self.publisher_ = self.create_publisher(PoseStamped, 'gt_poses_map', 10)

        # Create a TransformBroadcaster to publish tf
        self.tf_broadcaster = TransformBroadcaster(self)

    def pose_callback(self, msg):
        # Modify the frame_id
        new_frame_id = 'map'

        # Create a new PoseStamped message
        new_msg = PoseStamped()
        new_msg.header.stamp = msg.header.stamp
        new_msg.header.frame_id = new_frame_id
        new_msg.pose = msg.pose

        # Publish the modified PoseStamped
        self.publisher_.publish(new_msg)

        # Create and publish the TransformStamped based on the PoseStamped
        # From
        # https://github.com/robot-pesg/BotanicGarden/blob/01d55a9149190e0817c615d61aa0dbb0ff409010/calib/extrinsics/calib_chain.yaml#L151
        transform = TransformStamped()
        transform.header.stamp = new_msg.header.stamp
        transform.header.frame_id = new_msg.header.frame_id
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = new_msg.pose.position.x #+ 0.02725
        transform.transform.translation.y = new_msg.pose.position.y #- 0.0047
        transform.transform.translation.z = new_msg.pose.position.z #+ 0.99466
        transform.transform.rotation = new_msg.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = PoseRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

