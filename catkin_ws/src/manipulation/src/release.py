#! /usr/bin/env python

import sys
import rospy
import moveit_commander

from joint_cmds import Joint_Cmds


class Release:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.pick_joint_cmds = Joint_Cmds()

        # self.robot = moveit_commander.RobotCommander()
        # self.scene = moveit_commander.PlanningSceneInterface()
        # self.group_arm = moveit_commander.MoveGroupCommander("arm")
        # self.group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # # Get arm and gripper joint values
        # self.pick_joint_cmds.group_variable_values_arm_goal = self.group_arm.get_current_joint_values()
        # self.group_variable_values_gripper_close = (
        #     self.group_gripper.get_current_joint_values()
        # )

    def main(self):
        rospy.loginfo(
            "Reference frame to set end effector goal poses is: %s",
            self.group_arm.get_pose_reference_frame(),
        )

        rospy.loginfo("Go towards trash ..")
        self.pick_joint_cmds.dump_object()
        rospy.loginfo("release coke can..")
        self.pick_joint_cmds.open_gripper()
        rospy.loginfo("Retreating..")
        self.pick_joint_cmds.retreat()

        rospy.loginfo("Shuting Down ..")
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node("release_coke_to_trash", anonymous=True)
    release_object = Release()
    try:
        release_object.main()
    except rospy.ROSInterruptException:
        pass
