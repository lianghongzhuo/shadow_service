import rospkg
import os
import urdf_parser_py
import numpy as np
from urdf_parser_py.urdf import URDF

JOINT_NAMES = ["{}_WRJ2", "{}_WRJ1",
               "{}_FFJ4", "{}_FFJ3", "{}_FFJ2", "{}_FFJ1",
               "{}_MFJ4", "{}_MFJ3", "{}_MFJ2", "{}_MFJ1",
               "{}_RFJ4", "{}_RFJ3", "{}_RFJ2", "{}_RFJ1",
               "{}_LFJ5", "{}_LFJ4", "{}_LFJ3", "{}_LFJ2", "{}_LFJ1",
               "{}_THJ5", "{}_THJ4", "{}_THJ3", "{}_THJ2", "{}_THJ1"]


class JointLimit(object):
    def __init__(self, joint_limit_in_urdf):
        self.effort = joint_limit_in_urdf.effort
        self.velocity = joint_limit_in_urdf.velocity
        self.lower = joint_limit_in_urdf.lower
        self.upper = joint_limit_in_urdf.upper


class Joint(object):
    def __init__(self, joint_in_urdf):
        self.name = joint_in_urdf.name
        self.parent = joint_in_urdf.parent
        self.child = joint_in_urdf.child
        self.type = joint_in_urdf.joint_type
        self.axis = joint_in_urdf.axis

        if joint_in_urdf.limit is not None:
            self.limit = JointLimit(joint_in_urdf.limit)
        else:
            self.limit = None


class Robot(object):
    def __init__(self, robot_in_urdf):
        self.name = robot_in_urdf.name

        self.joints = []
        for joint in robot_in_urdf.joints:
            self.joints.append(Joint(joint))

        self.joint_map = {}
        self.parent_map = {}
        self.child_map = {}

        self.q_limit_map = {}

        for elem in self.joints:
            self.joint_map[elem.name] = elem
            self.parent_map[elem.child] = (elem.name, elem.parent)
            if elem.parent in self.child_map:
                self.child_map[elem.parent].append((elem.name, elem.child))
            else:
                self.child_map[elem.parent] = [(elem.name, elem.child)]

        for elem in self.joints:
            if elem.type == "revolute" or elem.type == "prismatic":
                self.q_limit_map[elem.name] = elem.limit


def get_joint_limits(hand_prefix="rh"):
    ros_pack = rospkg.RosPack()
    robot_dir = ros_pack.get_path("shadow_grasp_rl_ros")
    urdf_file = os.path.join(robot_dir, "urdf/ori_shadow_hand.urdf")
    with open(urdf_file, "r") as handle:
        urdf_strings = handle.read().replace("\n", "")
    robot_in_urdf = URDF.from_xml_string(urdf_strings)
    robot = Robot(robot_in_urdf=robot_in_urdf)
    limits = []
    for joint in JOINT_NAMES:
        joint = joint.format(hand_prefix)
        joint = robot.q_limit_map[joint]
        limits.append([joint.lower, joint.upper])
    limits = np.array(limits)
    return limits


if __name__ == "__main__":
    a = get_joint_limits()
