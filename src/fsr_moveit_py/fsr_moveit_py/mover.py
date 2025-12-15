#!/usr/bin/env python

from __future__ import print_function

from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

import sys
import copy
import math
from pymoveit2 import MoveIt2, MoveIt2State
# import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, RobotTrajectory
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list

from fsr_moveit.srv import PickAndPlaceService

if True:
    def planCompat(plan):
        return RobotTrajectory(joint_trajectory=plan)
else:
    raise NotImplementedError()

class FSR_MoveIt_Server(Node):
    def __init__(self):
        super().__init__('fsr_moveit_server')
        self._mover_srv = self.create_service(PickAndPlaceService, 'fsr_moveit', self._plan_pick_and_place)
    
    """
    Creates a pick and place plan using the four states below.
    
    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position

    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved

    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
    """
    def _plan_pick_and_place(self, req, res):
        self.get_logger().info("Recieved request to plan pick-and-place trajectory...")

        callback_group = ReentrantCallbackGroup()

        joint_names = req.joints_input.joint_names
        group_name = req.pnp_input.group_name
        end_effector_name = req.pnp_input.end_effector_name
        base_link_name = req.pnp_input.base_link_name
        max_velocity = req.pnp_input.max_velocity
        max_acceleration = req.pnp_input.max_acceleration
        planner_id = "RRTConnectkConfigDefault"

        move_group = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name=base_link_name,
            end_effector_name=end_effector_name,
            group_name=group_name,
            callback_group=callback_group,
        )
        move_group.planner_id = ( planner_id )

        # Spin the node in background thread(s) and wait a bit for initialization
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        self.create_rate(1.0).sleep()

        current_robot_joint_configuration = req.joints_input.joints

        # Pre grasp - position gripper directly above target object
        pre_grasp_pose = self._plan_trajectory(joint_names, move_group, req.pick_pose, current_robot_joint_configuration, max_velocity, max_acceleration)

         # If the trajectory has no points, planning has failed and we return an empty response
        if not pre_grasp_pose.joint_trajectory.points:
            return res

        previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

        # Grasp - lower gripper so that fingers are on either side of object
        pick_pose = copy.deepcopy(req.pick_pose)
        pick_pose.position.z -= req.pnp_input.pick_pose_z # Static value coming from Unity
        grasp_pose = self._plan_trajectory(joint_names, move_group, pick_pose, previous_ending_joint_angles, max_velocity, max_acceleration)

        if not pre_grasp_pose.joint_trajectory.points:
            return res

        previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

        # Pick Up - raise gripper back to the pre grasp position
        pick_up_pose = self._plan_trajectory(joint_names, move_group, req.pick_pose, previous_ending_joint_angles, max_velocity, max_acceleration)

        if not pick_up_pose.joint_trajectory.points:
            return res

        previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

        # Place - move gripper to desired placement position
        place_pose = self._plan_trajectory(joint_names, move_group, req.place_pose, previous_ending_joint_angles, max_velocity, max_acceleration)

        if not place_pose.joint_trajectory.points:
            return res

        # If trajectory planning worked for all pick and place stages, add plan to response
        res.trajectories.append(pre_grasp_pose)
        res.trajectories.append(grasp_pose)
        res.trajectories.append(pick_up_pose)
        res.trajectories.append(place_pose)

        self.get_logger().info("UR5e cobot trajectories generated. Have a nice day!")

        return res
    
    """
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
    """
    def _plan_trajectory(self, joint_names, move_group, destination_pose, start_joint_angles, max_velocity = 0.5, max_acceleration = 0.5):
        start_joint_angles = start_joint_angles.tolist()

        current_joint_state = JointState()
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_angles

        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = current_joint_state

        move_group.max_velocity = max_velocity
        move_group.max_acceleration = max_acceleration

        plan = move_group.plan(start_joint_state=current_joint_state, pose=destination_pose, cartesian=True)

        if not plan:
            exception_str = """
                Trajectory could not be planned for a destination of {} with starting joint angles {}.
                Please make sure target and destination are reachable by the robot.
            """.format(destination_pose, destination_pose)
            raise Exception(exception_str)

        return planCompat(plan)

def main(args=None):
    rclpy.init(args=args)

    moveit_server = FSR_MoveIt_Server()

    try:
        rclpy.spin(moveit_server)
    except KeyboardInterrupt:
        pass
    finally:
        moveit_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()