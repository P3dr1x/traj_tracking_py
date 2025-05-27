"""
Publish PoseStamped of ee_gripper_link w.r.t base_link at 100 Hz
Topic: /ee_pose
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

class EEPosePublisher(Node):
    def __init__(self):
        super().__init__("ee_pose_publisher")

        # ---- parameters (easy to remap via launch) ----
        # self.declare_parameter("parent_frame", "base_link")
        # self.declare_parameter("child_frame",  "ee_gripper_link")
        # parent = self.get_parameter("parent_frame").get_parameter_value().string_value
        # child  = self.get_parameter("child_frame").get_parameter_value().string_value
        self.parent_frame = "mobile_wx250s/base_link"
        self.child_frame  = "mobile_wx250s/ee_gripper_link"
        # ---- tf2 ----
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- publisher ----
        self.pub = self.create_publisher(PoseStamped, "/ee_pose", 10)

        # ---- timer 100 Hz ----
        self.timer = self.create_timer(0.01, self.timer_cb)
        self.get_logger().info(f"Publishing {self.child_frame} pose w.r.t {self.parent_frame} on /ee_pose")

    def timer_cb(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )

            msg = PoseStamped()
            msg.header = trans.header
            msg.pose.position.x = trans.transform.translation.x
            msg.pose.position.y = trans.transform.translation.y
            msg.pose.position.z = trans.transform.translation.z
            msg.pose.orientation = trans.transform.rotation
            self.pub.publish(msg)

        except Exception as e:  # tf2_ros.LookupException, ConnectivityException, ExtrapolationException
            self.get_logger().debug(f"TF unavailable: {e}")

def main():
    rclpy.init()
    node = EEPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
