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
    def __init__(self, safe_mode):
        self.hand_type = rospy.get_param("hand_motion_server/hand_type")
        if self.hand_type == "right_hand":
            self.name_prefix = "rh"
            self.hand_group = "right_hand"
        elif self.hand_type == "left_hand":
            self.name_prefix = "lh"
            self.hand_group = "hand"
        else:
            raise NotImplementedError
        self.safe_mode = safe_mode
        self.hand_commander = SrHandCommander(name=self.hand_group)
        self.hand_limits = get_joint_limits()
        get_correct_joints = False
        while not get_correct_joints:
            self.hand_pos = self.hand_commander.get_joints_position()
            if "{}_MFJ2".format(self.name_prefix) in self.hand_pos.keys():
                get_correct_joints = True
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
        self.hand_pos.update({self.name_prefix + "_WRJ2": goal[0]})
        self.hand_pos.update({self.name_prefix + "_WRJ1".format(self.name_prefix): goal[1]})

        # first finger
        self.hand_pos.update({self.name_prefix + "_FFJ4": goal[2]})
        self.hand_pos.update({self.name_prefix + "_FFJ3": goal[3]})
        self.hand_pos.update({self.name_prefix + "_FFJ2": goal[4]})

        # middle finger
        self.hand_pos.update({self.name_prefix + "_MFJ4": goal[6]})
        self.hand_pos.update({self.name_prefix + "_MFJ3": goal[7]})
        self.hand_pos.update({self.name_prefix + "_MFJ2": goal[8]})

        # ring finger
        self.hand_pos.update({self.name_prefix + "_RFJ4": goal[10]})
        self.hand_pos.update({self.name_prefix + "_RFJ3": goal[11]})
        self.hand_pos.update({self.name_prefix + "_RFJ2": goal[12]})

        # little finger
        self.hand_pos.update({self.name_prefix + "_LFJ5": goal[14]})
        self.hand_pos.update({self.name_prefix + "_LFJ4": goal[15]})
        self.hand_pos.update({self.name_prefix + "_LFJ3": goal[16]})
        self.hand_pos.update({self.name_prefix + "_LFJ2": goal[17]})

        # thumb
        self.hand_pos.update({self.name_prefix + "_THJ5": goal[19]})
        self.hand_pos.update({self.name_prefix + "_THJ4": goal[20]})
        self.hand_pos.update({self.name_prefix + "_THJ3": goal[21]})
        self.hand_pos.update({self.name_prefix + "_THJ2": goal[22]})

        # special joints for our left hand
        if self.hand_type == "left_hand":
            self.hand_pos.update({self.name_prefix + "_FFJ1".format(self.name_prefix): goal[5]})
            self.hand_pos.update({self.name_prefix + "_MFJ1".format(self.name_prefix): goal[9]})
            self.hand_pos.update({self.name_prefix + "_RFJ1".format(self.name_prefix): goal[13]})
            self.hand_pos.update({self.name_prefix + "_LFJ1".format(self.name_prefix): goal[18]})
            self.hand_pos.update({self.name_prefix + "_THJ1".format(self.name_prefix): goal[23]})
        rospy.loginfo("getting shadow hand command {}".format(self.hand_pos))

        if self.safe_mode:
            self.hand_commander.move_to_joint_value_target(self.hand_pos, angle_degrees=False)
        else:
            self.hand_commander.move_to_joint_value_target_unsafe(self.hand_pos, 0.3, False, angle_degrees=False)
        rospy.loginfo("Next one please ---->")
        return ShadowCommanderSrvResponse(True)


if __name__ == "__main__":
    rospy.init_node("shadow_commander_server")
    server = ShadowCommanderServer(safe_mode=True)
    rospy.spin()
