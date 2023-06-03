#! /usr/bin/env python

import sys
import rospy
import moveit_commander

from joint_cmds import Joint_Cmds


class Camera_Alignment:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.pick_joint_cmds = Joint_Cmds()

    def main(self):
        rospy.loginfo("Orient Camera towards the table..")
        self.pick_joint_cmds.camera_alignment()

        rospy.loginfo("Shuting Down ..")
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node("orient_camera_node", anonymous=True)
    orient_camera_object = Camera_Alignment()
    try:
        orient_camera_object.main()
    except rospy.ROSInterruptException:
        pass
