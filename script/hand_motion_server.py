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
from sr_robot_commander.sr_hand_commander import SrHandCommander


class ShadowCommanderServer:
    def __init__(self):
        self.hand_type = rospy.get_param("~hand_type")
        self.safe_mode = rospy.get_param("~shadow_hand_safe_mode")
        if self.hand_type == "right_hand":
            name_prefix = "rh"
            self.hand_group = "right_hand"
        elif self.hand_type == "left_hand":
            name_prefix = "lh"
            self.hand_group = "hand"
        else:
            raise NotImplementedError
        if self.safe_mode:
            self.hand_commander = MoveGroupCommander(self.hand_group)
        else:
            self.hand_commander = SrHandCommander(name=self.hand_group)

        self.hand_limits = get_joint_limits()
        self.joint_names = []
        for joint in JOINT_NAMES:
            self.joint_names.append(joint.format(name_prefix))
        rospy.Service("shadow_commander_service", ShadowCommanderSrv, self.service_callback)

    def clip_hand_pos(self, hand_pos):
        if_lower_limit = hand_pos <= self.hand_limits[:, 0]
        if_upper_limit = hand_pos >= self.hand_limits[:, 1]
        hand_pos = if_lower_limit * self.hand_limits[:, 0] + np.logical_not(if_lower_limit) * hand_pos
        hand_pos = if_upper_limit * self.hand_limits[:, 1] + np.logical_not(if_upper_limit) * hand_pos
        return hand_pos

    def service_callback(self, msg):
        goal = msg.joint_positions.data
        goal = self.clip_hand_pos(goal)
        rospy.loginfo("getting shadow hand command {}".format(goal))
        hand_joint_positions = {}
        for i, joint in enumerate(self.joint_names):
            hand_joint_positions[joint] = goal[i]
        if self.safe_mode:
            self.hand_commander.set_start_state_to_current_state()
            self.hand_commander.set_joint_value_target(hand_joint_positions)
            self.hand_commander.go(wait=True)
        else:
            self.hand_commander.move_to_joint_value_target_unsafe(hand_joint_positions, 0.5, wait=True,
                                                                  angle_degrees=False)
        rospy.loginfo("Next one please ---->")
        return ShadowCommanderSrvResponse(True)


if __name__ == "__main__":
    rospy.init_node("shadow_commander_server", anonymous=True)
    server = ShadowCommanderServer()
    rospy.spin()
