#!/usr/bin/env python

from __future__ import print_function

import sys
import copy
import math
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from pymoveit2 import MoveIt2
from fsr_moveit_py.trajectory_planner import TrajectoryPlanner
from fsr_moveit.srv import PickAndPlaceService, MoveService
from fsr_moveit.srv._pick_and_place_service import PickAndPlaceService_Request, PickAndPlaceService_Response
from fsr_moveit.srv._move_service import MoveService_Request, MoveService_Response


class FSR_MoveIt_Server(Node):
    def __init__(self):
        super().__init__('fsr_moveit_server')
        self._pnp_srv = self.create_service(PickAndPlaceService, 'fsr_moveit_pick_and_place_srv', self._service_pick_and_place)
        self._move_srv = self.create_service(MoveService, 'fsr_moveit_move_srv', self._service_move)
    
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
    def _service_pick_and_place(self, req : PickAndPlaceService_Request, res : PickAndPlaceService_Response) -> PickAndPlaceService_Response:
        self.get_logger().info("Recieved request to plan pick-and-place trajectory...")

        callback_group = ReentrantCallbackGroup()

        joint_names = req.joints_input.joint_names
        group_name = req.group.group_name
        end_effector_name = req.group.end_effector_name
        base_link_name = req.group.base_link_name
        max_velocity = req.pars.max_velocity
        max_acceleration = req.pars.max_acceleration
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
        trajectory_planner : TrajectoryPlanner = TrajectoryPlanner(max_velocity, max_acceleration)

        # Spin the node in background thread(s) and wait a bit for initialization
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        self.create_rate(1.0).sleep()

        current_robot_joint_configuration = req.joints_input.joints

        # Pre grasp - position gripper directly above target object
        pre_grasp_pose = trajectory_planner.plan_trajectory(joint_names, move_group, req.pars.pick_pose, current_robot_joint_configuration)

         # If the trajectory has no points, planning has failed and we return an empty response
        if not pre_grasp_pose.joint_trajectory.points:
            return res

        previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

        # Grasp - lower gripper so that fingers are on either side of object
        pick_pose = copy.deepcopy(req.pars.pick_pose)
        pick_pose.position.z -= req.pars.pick_pose_z # Static value coming from Unity
        grasp_pose = trajectory_planner.plan_trajectory(joint_names, move_group, pick_pose, previous_ending_joint_angles)

        if not pre_grasp_pose.joint_trajectory.points:
            return res

        previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

        # Pick Up - raise gripper back to the pre grasp position
        pick_up_pose = trajectory_planner.plan_trajectory(joint_names, move_group, req.pars.pick_pose, previous_ending_joint_angles)

        if not pick_up_pose.joint_trajectory.points:
            return res

        previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

        # Place - move gripper to desired placement position
        place_pose = copy.deepcopy(req.pars.place_pose)
        place_pose.position.z -= req.pars.place_pose_z
        release_pose = trajectory_planner.plan_trajectory(joint_names, move_group, place_pose, previous_ending_joint_angles)

        if not release_pose.joint_trajectory.points:
            return res

        # If trajectory planning worked for all pick and place stages, add plan to response
        res.trajectories.append(pre_grasp_pose)
        res.trajectories.append(grasp_pose)
        res.trajectories.append(pick_up_pose)
        res.trajectories.append(release_pose)

        self.get_logger().info("Trajectories generated. Have a nice day!")

        return res

    def _service_move(self, req : MoveService_Request, res : MoveService_Response) -> MoveService_Response:
        self.get_logger().info("Recieved request to plan trajectory...")

        callback_group = ReentrantCallbackGroup()

        joint_names = req.joints_input.joint_names
        group_name = req.group.group_name
        end_effector_name = req.group.end_effector_name
        base_link_name = req.group.base_link_name
        max_velocity = req.pars.max_velocity
        max_acceleration = req.pars.max_acceleration
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
        trajectory_planner : TrajectoryPlanner = TrajectoryPlanner(max_velocity, max_acceleration)

        current_robot_joint_configuration = req.joints_input.joints
        target_pose = copy.deepcopy(req.pars.target_pose)
        target_pose.position.z -= req.ee_offset
        goal_pose = trajectory_planner.plan_trajectory(joint_names, move_group, target_pose, current_robot_joint_configuration)

         # If the trajectory has no points, planning has failed and we return an empty response
        if not goal_pose.joint_trajectory.points:
            return res

        res.trajectory = goal_pose
        return res


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