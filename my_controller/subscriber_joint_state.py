"""A demo subscriber to joint_states topic which prints the joint states on the console."""
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.joint_state_sub = self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 10
            )
        self.i = 0 # no. of received message in a cycle

    def joint_state_callback(self, msg):
        if self.i == 1000:
            self.get_logger().info(f"Positions: {msg.position}")
            self.get_logger().info(f"Velocity: {msg.velocity}")
            self.get_logger().info(f"Effort: {msg.effort}")
            self.i = 0
        
        self.i += 1
    
def main(args=None):
    rclpy.init(args=args)

    joint_state_sub = JointStateSubscriber()

    rclpy.spin(joint_state_sub)
    joint_state_sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()