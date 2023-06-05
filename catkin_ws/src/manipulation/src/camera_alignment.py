#! /usr/bin/env python

import sys
import rospy
import moveit_commander

from joint_cmds import JointCommands


class CameraAlignment:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.joint_cmds = JointCommands()

    def main(self):
        rospy.loginfo("Orient the camera to perceive the environment..")
        self.joint_cmds.camera_alignment()
        rospy.loginfo("Shutting Down..")
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node("camera_alignment_node", anonymous=True)
    orient_camera_object = CameraAlignment()
    try:
        orient_camera_object.main()
    except rospy.ROSInterruptException:
        pass
