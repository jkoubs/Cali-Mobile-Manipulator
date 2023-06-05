#! /usr/bin/env python

import sys
import rospy
import moveit_commander

from joint_cmds import JointCommands


class Release:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.joint_cmds = JointCommands()

    def main(self):
        rospy.loginfo("Go towards trash..")
        self.joint_cmds.dump_object()
        rospy.loginfo("Dump coke can..")
        self.joint_cmds.open_gripper()
        rospy.loginfo("Retreating..")
        self.joint_cmds.retreat()

        rospy.loginfo("Shutting Down..")
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node("release_node", anonymous=True)
    release_object = Release()
    try:
        release_object.main()
    except rospy.ROSInterruptException:
        pass
