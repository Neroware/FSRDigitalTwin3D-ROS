from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, RobotTrajectory
from geometry_msgs.msg import Pose

from pymoveit2 import MoveIt2

if True:
    def planCompat(plan):
        return RobotTrajectory(joint_trajectory=plan)
else:
    raise NotImplementedError()

class TrajectoryPlanner:
        def __init__(self, max_velocity : float = 0.5, max_acceleration : float = 0.5):
            self.max_velocity : float = max_velocity
            self.max_acceleration : float = max_acceleration

        """
        Given the start angles of the robot, plan a trajectory that ends at the destination pose.
        """
        def plan_trajectory(self, joint_names : list[str], move_group : MoveIt2, destination_pose : Pose, start_joint_angles : list[float]):
            start_joint_angles = start_joint_angles.tolist()

            current_joint_state = JointState()
            current_joint_state.name = joint_names
            current_joint_state.position = start_joint_angles

            moveit_robot_state = RobotState()
            moveit_robot_state.joint_state = current_joint_state

            move_group.max_velocity = self.max_velocity
            move_group.max_acceleration = self.max_acceleration

            plan = move_group.plan(start_joint_state=current_joint_state, pose=destination_pose, cartesian=True)

            if not plan:
                exception_str = """
                    Trajectory could not be planned for a destination of {} with starting joint angles {}.
                    Please make sure target and destination are reachable by the robot.
                """.format(destination_pose, destination_pose)
                raise Exception(exception_str)

            return planCompat(plan)