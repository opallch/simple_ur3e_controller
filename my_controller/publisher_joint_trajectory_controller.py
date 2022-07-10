"""
The publisher publishes the trajectories to the <controller>/joint_trajectory topic
which will be executed after a (fake) robot is connected.
"""
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointsParamNotSetError(Exception):
    def __init__(self):
        self.msg = "Please define the 'joints' parameters in the corresponding yaml file!"

class GoalNotSetError(Exception):
    def __init__(self, name):
        self.msg = f"Values for goal {name} not set!"

class PublisherJointTrajectory(Node):
    def __init__(self):
        # define the name of the node
        super().__init__("publisher_joint_trajectory_controller")
        
        # declare ROS parameters; the values will be substituted by the values in the yaml file
        param_names = ["controller_name", "wait_sec_between_publish", "goal_names", "joints"]
        
        for param_name in param_names:
            self.declare_parameter(param_name, None)
        
        # creates attributes with the values from `config/publisher_config.yaml`
        controller_name, wait_sec_between_publish, self.goal_names, self.joints  = \
            [param.value for param in self.get_parameters(param_names)]
        
        self.goals = self.get_goals_from_ros_param()

        # check if the params in `config/publisher_config.yaml` are set properly
        if self.joints is None or len(self.joints) == 0:
             raise JointsParamNotSetError()
        
        # set up publisher
        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.get_logger().info(
            f"Publishing {len(self.goal_names)} goals on topic '{publish_topic}' every {wait_sec_between_publish} s"
            )
        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def get_goals_from_ros_param(self):
        goals = []
        
        for goal_name in self.goal_names:
            # check if the goal is set in yml
            self.declare_parameter(goal_name)
            goal = self.get_parameter(goal_name).value
            if goal is None or len(goal) == 0:
                raise GoalNotSetError(goal_name)

            # parse the values to float
            float_goal = []
            for value in goal:
                float_goal.append(float(value))
            goals.append(float_goal)

        return goals
    
    def timer_callback(self):

        traj = JointTrajectory()
        traj.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goals[self.i]
        point.time_from_start = Duration(sec=4)

        traj.points.append(point)
        self.publisher_.publish(traj)

        self.i += 1
        self.i %= len(self.goals)


def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()