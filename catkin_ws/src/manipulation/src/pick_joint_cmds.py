#! /usr/bin/env python

import sys
import rospy
import moveit_commander


class Pick_Place:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_arm = moveit_commander.MoveGroupCommander("arm")
        self.group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Get arm and gripper joint values
        self.group_variable_values_arm_goal = self.group_arm.get_current_joint_values()
        self.group_variable_values_gripper_close = (
            self.group_gripper.get_current_joint_values()
        )

    def execute_arm_cmds(self):
        """
        Executes the group "arm" commands
        """
        self.group_arm.set_joint_value_target(self.group_variable_values_arm_goal)
        self.group_arm.go(wait=True)
        rospy.sleep(2)

    def execute_gripper_cmds(self):
        """
        Executes the group "gripper" commands
        """
        self.group_gripper.set_joint_value_target(
            self.group_variable_values_gripper_close
        )
        self.group_gripper.go(wait=True)
        rospy.sleep(2)

    def open_gripper(self):
        """
        Step 1: Open the Gripper
        """
        self.group_variable_values_gripper_close[0] = 1.57
        self.execute_gripper_cmds()

    def pregrasp(self):
        """
        Step 2: Pregrasp
        """
        self.group_variable_values_arm_goal[0] = 0.15
        self.group_variable_values_arm_goal[1] = 0.23
        self.group_variable_values_arm_goal[2] = 1.00
        self.group_variable_values_arm_goal[3] = 1.90
        self.group_variable_values_arm_goal[4] = -0.19
        self.execute_arm_cmds()

    def grasp(self):
        """
        Step 3: Grasp
        """
        self.group_variable_values_gripper_close[0] = 0.60
        self.execute_gripper_cmds()

    def retreat(self):
        """
        Step 4: Retreat
        """
        self.group_variable_values_arm_goal[0] = 0.0
        self.group_variable_values_arm_goal[1] = 0.45
        self.group_variable_values_arm_goal[2] = -0.82
        self.group_variable_values_arm_goal[3] = 1.95
        self.group_variable_values_arm_goal[4] = 0.0
        self.execute_arm_cmds()

    def main(self):
        """
        Performs the pick mission
        Consists of picking up the coke can from the table
        For the entire mission, it uses the joint commands
        """
        rospy.loginfo(
            "Reference frame to set end effector goal poses is: %s",
            self.group_arm.get_pose_reference_frame(),
        )

        rospy.loginfo("Open Gripper ..")
        pick_place_object.open_gripper()
        rospy.loginfo("Pregrasp..")
        pick_place_object.pregrasp()
        rospy.loginfo("Grasp the object..")
        pick_place_object.grasp()
        rospy.loginfo("Retreating..")
        pick_place_object.retreat()

        rospy.loginfo("Shuting Down ..")
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node("pick_place_node_joint_values", anonymous=True)
    pick_place_object = Pick_Place()
    try:
        pick_place_object.main()
    except rospy.ROSInterruptException:
        pass