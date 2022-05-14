#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Hongzhuo Liang 
# E-mail     : liang@informatik.uni-hamburg.de
# Description: 
# Date       : 09/02/2021: 12:04
# File Name  : hand_motion
from __future__ import print_function
import rospy
from moveit_commander import MoveGroupCommander
from shadow_service.srv import ShadowCommanderSrv, ShadowCommanderSrvResponse
from get_joint_limits import get_joint_limits, JOINT_NAMES
import numpy as np
import math
import copy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib import SimpleActionClient


class ShadowCommanderServer:
    def __init__(self):
        self.hand_type = rospy.get_param("~hand_type")
        self.safe_mode = rospy.get_param("~shadow_hand_safe_mode")
        if self.hand_type == "right_hand":
            self.name_prefix = "rh"
            self.hand_group = "right_hand"
        elif self.hand_type == "left_hand":
            self.name_prefix = "lh"
            self.hand_group = "hand"
        else:
            raise NotImplementedError
        self.controller_list = ["hand/{}_trajectory_controller".format(self.name_prefix),
                                "hand/{}_wr_trajectory_controller".format(self.name_prefix)]

        if self.safe_mode:
            self.moveit_commander = MoveGroupCommander(self.hand_group)
        else:
            self._clients = {}
            self._set_up_action_client()
        self.hand_limits = get_joint_limits()
        self.joint_names = []
        for joint in JOINT_NAMES:
            self.joint_names.append(joint.format(self.name_prefix))
        self.controller_joints = {self.controller_list[1]: self.joint_names[:2],
                                  self.controller_list[0]: self.joint_names[2:]}
        rospy.Service("shadow_commander_service", ShadowCommanderSrv, self.service_callback)

    def clip_hand_pos(self, hand_pos):
        if_lower_limit = hand_pos <= self.hand_limits[:, 0]
        if_upper_limit = hand_pos >= self.hand_limits[:, 1]
        hand_pos = if_lower_limit * self.hand_limits[:, 0] + np.logical_not(if_lower_limit) * hand_pos
        hand_pos = if_upper_limit * self.hand_limits[:, 1] + np.logical_not(if_upper_limit) * hand_pos
        return hand_pos

    def service_callback(self, msg):
        goal = msg.joint_positions.data
        time_limit = msg.time_limit.data
        goal = self.clip_hand_pos(goal)
        rospy.loginfo("getting shadow hand command {}".format(goal))
        hand_joint_positions = {}
        for i, joint in enumerate(self.joint_names):
            hand_joint_positions[joint] = goal[i]
        if self.safe_mode:
            self.moveit_commander.set_start_state_to_current_state()
            self.moveit_commander.set_joint_value_target(hand_joint_positions)
            self.moveit_commander.go(wait=True)
        else:
            self.move_to_joint_value_target_unsafe(hand_joint_positions, time_limit, wait=True, angle_degrees=False)
        rospy.loginfo("Next one please ---->")
        return ShadowCommanderSrvResponse(True)

    def _set_up_action_client(self):
        """
        Sets up an action client to communicate with the trajectory controller.
        """
        self._action_running = {}
        for controller_name in self.controller_list:
            self._action_running[controller_name] = False
            service_name = controller_name + "/follow_joint_trajectory"
            self._clients[controller_name] = SimpleActionClient(service_name, FollowJointTrajectoryAction)
            if self._clients[controller_name].wait_for_server(timeout=rospy.Duration(4)) is False:
                err_msg = 'Failed to connect to action server ({}) in 4 sec'.format(service_name)
                rospy.logwarn(err_msg)

    def move_to_joint_value_target_unsafe(self, joint_states, time=0.002, wait=True, angle_degrees=False):
        """
        Set target of the robot's links and moves to it.
        @param joint_states - dictionary with joint name and value. It can
        contain only joints values of which need to be changed.
        @param time - time in s (counting from now) for the robot to reach the
        target (it needs to be greater than 0.0 for it not to be rejected by
        the trajectory controller).
        @param wait - should method wait for movement end or not.
        @param angle_degrees - are joint_states in degrees or not.
        """
        goals = {}
        joint_states_cpy = copy.deepcopy(joint_states)

        if angle_degrees:
            joint_states_cpy.update((joint, math.radians(i)) for joint, i in joint_states_cpy.items())

        for i, controller in enumerate(self._clients):
            controller_joints = self.controller_joints[controller]
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = []
            point = JointTrajectoryPoint()
            point.positions = []

            for x in joint_states_cpy.keys():
                if x in controller_joints:
                    goal.trajectory.joint_names.append(x)
                    point.positions.append(joint_states_cpy[x])

            point.time_from_start = rospy.Duration.from_sec(time)
            goal.trajectory.points = [point]
            goals[controller] = goal
            self._action_running[controller] = True
            self._clients[controller].send_goal(
                goals[controller], lambda terminal_state, result: self._action_done_cb(client, terminal_state, result))

        if not wait:
            return

        for client in self._clients.keys():
            if not self.action_is_running(client):
                continue
            if not self._clients[client].wait_for_result():
                rospy.loginfo("Trajectory not completed")

    def action_is_running(self, controller=None):
        if controller is not None:
            return self._action_running[controller]

        for controller_running in self._action_running.values():
            if controller_running:
                return True
        return False

    def _action_done_cb(self, controller, terminal_state, result):
        self._action_running[controller] = False


if __name__ == "__main__":
    rospy.init_node("shadow_commander_server", anonymous=True)
    server = ShadowCommanderServer()
    rospy.spin()
