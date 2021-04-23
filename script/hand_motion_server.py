#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Hongzhuo Liang 
# E-mail     : liang@informatik.uni-hamburg.de
# Description: 
# Date       : 09/02/2021: 12:04
# File Name  : hand_motion
from __future__ import print_function
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from shadow_service.srv import ShadowCommanderSrv, ShadowCommanderSrvResponse
from get_joint_limits import get_joint_limits
import numpy as np


class ShadowCommanderServer:
    def __init__(self):
        self.hand_type = rospy.get_param("hand_motion_server/hand_type")
        self.safe_mode = rospy.get_param("hand_motion_server/shadow_hand_safe_mode")
        if self.hand_type == "right_hand":
            self.name_prefix = "rh"
            self.hand_group = "right_hand"
        elif self.hand_type == "left_hand":
            self.name_prefix = "lh"
            self.hand_group = "hand"
        else:
            raise NotImplementedError
        self.hand_commander = SrHandCommander(name=self.hand_group)
        self.hand_limits = get_joint_limits()
        rospy.Service("shadow_commander_service", ShadowCommanderSrv, self.service_callback)

    def clip_hand_pos(self, hand_pos):
        if_lower_limit = hand_pos <= self.hand_limits[:, 0]
        if_upper_limit = hand_pos >= self.hand_limits[:, 1]
        hand_pos = if_lower_limit * self.hand_limits[:, 0] + np.logical_not(if_lower_limit) * hand_pos
        hand_pos = if_upper_limit * self.hand_limits[:, 1] + np.logical_not(if_upper_limit) * hand_pos
        return hand_pos

    def service_callback(self, joints_msg):
        goal = joints_msg.joint_positions.data
        goal = self.clip_hand_pos(goal)
        # wrist
        hand_joint_positions = {
                                # wrist
                                self.name_prefix + "_WRJ2": goal[0],
                                self.name_prefix + "_WRJ1": goal[1],
                                # first finger
                                self.name_prefix + "_FFJ4": goal[2],
                                self.name_prefix + "_FFJ3": goal[3],
                                self.name_prefix + "_FFJ2": goal[4],
                                self.name_prefix + "_FFJ1": goal[5],
                                # middle finger
                                self.name_prefix + "_MFJ4": goal[6],
                                self.name_prefix + "_MFJ3": goal[7],
                                self.name_prefix + "_MFJ2": goal[8],
                                self.name_prefix + "_MFJ1": goal[9],
                                # ring finger
                                self.name_prefix + "_RFJ4": goal[10],
                                self.name_prefix + "_RFJ3": goal[11],
                                self.name_prefix + "_RFJ2": goal[12],
                                self.name_prefix + "_RFJ1": goal[13],
                                # little finger
                                self.name_prefix + "_LFJ5": goal[14],
                                self.name_prefix + "_LFJ4": goal[15],
                                self.name_prefix + "_LFJ3": goal[16],
                                self.name_prefix + "_LFJ2": goal[17],
                                self.name_prefix + "_LFJ1": goal[18],
                                # thumb
                                self.name_prefix + "_THJ5": goal[19],
                                self.name_prefix + "_THJ4": goal[20],
                                self.name_prefix + "_THJ3": goal[21],
                                self.name_prefix + "_THJ2": goal[22],
                                self.name_prefix + "_THJ1": goal[23],
                                }

        rospy.loginfo("getting shadow hand command {}".format(hand_joint_positions))
        if self.safe_mode:
            self.hand_commander.move_to_joint_value_target(hand_joint_positions, angle_degrees=False)
        else:
            self.hand_commander.move_to_joint_value_target_unsafe(hand_joint_positions, 0.3, False, angle_degrees=False)
        rospy.loginfo("Next one please ---->")
        return ShadowCommanderSrvResponse(True)


if __name__ == "__main__":
    rospy.init_node("shadow_commander_server")
    server = ShadowCommanderServer()
    rospy.spin()
