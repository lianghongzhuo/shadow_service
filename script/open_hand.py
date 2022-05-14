#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Hongzhuo Liang 
# E-mail     : liang@informatik.uni-hamburg.de
# Description: 
# Date       : 22/05/2021: 10:03
# File Name  : open hand
from __future__ import print_function
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from get_joint_limits import FINGER_JOINT_NAMES


def main(name_prefix="lh"):
    controller_name = "/hand/{}_trajectory_controller/command".format(name_prefix)
    hand_pub = rospy.Publisher(controller_name, JointTrajectory, queue_size=1, latch=True)
    # simple version of shadow hand commander function: move_to_joint_value_target_unsafe()
    joint_names = []
    for joint in FINGER_JOINT_NAMES:
        joint_names.append(joint.format(name_prefix))
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    trajectory.points.append(JointTrajectoryPoint())
    goal = [0] * len(FINGER_JOINT_NAMES)
    trajectory.points[0].positions = goal
    trajectory.points[0].time_from_start.secs = 1
    hand_pub.publish(trajectory)
    rospy.sleep(1)
    rospy.loginfo("finish open hand")


if __name__ == "__main__":
    rospy.init_node("shadow_open_hand_script", anonymous=True)
    main()
