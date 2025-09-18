#!/usr/bin/env python3
"""
    Subscribes to SourceDestination topic.
    Uses MoveIt to compute a trajectory from the target to the destination.
    Trajectory is then published to PickAndPlaceTrajectory topic.
"""
import rclpy
from rclpy.node import Node

from ur5e_moveit.msg import UR5eMoveitJoints, UR5eTrajectory
from moveit_msgs.msg import RobotTrajectory


class TrajectorySubscriber(Node):
    def __init__(self):
        super().__init__('trajectory_subscriber')
        self.subscription = self.create_subscription(
            UR5eMoveitJoints,
            '/ur5e_joints',
            self.callback,
            10  # QoS History depth
        )

    def callback(self, msg):
        self.get_logger().info(f"I heard:\n{msg}")


def main(args=None):
    rclpy.init(args=args)

    trajectory_subscriber = TrajectorySubscriber()

    try:
        rclpy.spin(trajectory_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()