#! /usr/bin/env python

import sys
import rospy
import moveit_commander


class JointCommands:
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

    def set_group_config(self, planning_time, position_tol, orientration_tol):
        """
        Sets groups configuration for MoveIt
        """
        self.group_arm.allow_replanning(True)
        self.group_arm.set_planning_time(planning_time)
        self.group_arm.set_goal_position_tolerance(position_tol)
        self.group_arm.set_goal_orientation_tolerance(orientration_tol)

        print("Reference frame: %s" % self.group_arm.get_planning_frame())

        print("End effector: %s" % self.group_arm.get_end_effector_link())

        print("Robot Groups: %s" % self.robot.get_group_names())

        print("Current Joint Values:")
        print(self.group_arm.get_current_joint_values())

        print("Current Pose:")
        print(self.group_arm.get_current_pose())

        print("Robot State:")
        print(self.robot.get_current_state())

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
        Open the Gripper
        """
        self.group_variable_values_gripper_close[0] = 1.57
        self.execute_gripper_cmds()

    def pregrasp(self):
        """
        Pregrasp
        """
        self.group_variable_values_arm_goal[0] = 0.15
        self.group_variable_values_arm_goal[1] = 0.23
        self.group_variable_values_arm_goal[2] = 1.00
        self.group_variable_values_arm_goal[3] = 1.90
        self.group_variable_values_arm_goal[4] = -0.19
        self.execute_arm_cmds()

    def grasp(self):
        """
        Grasp
        """
        self.group_variable_values_gripper_close[0] = 0.60
        self.execute_gripper_cmds()

    def retreat(self):
        """
        Retreat
        """
        self.group_variable_values_arm_goal[0] = 0.0
        self.group_variable_values_arm_goal[1] = 0.45
        self.group_variable_values_arm_goal[2] = -0.82
        self.group_variable_values_arm_goal[3] = 1.95
        self.group_variable_values_arm_goal[4] = 0.0
        self.execute_arm_cmds()

    def camera_alignment(self):
        """
        Orient Camera in order to visualize our environment
        """
        self.group_variable_values_arm_goal[0] = 0.0
        self.group_variable_values_arm_goal[1] = -0.35
        self.group_variable_values_arm_goal[2] = 0.73
        self.group_variable_values_arm_goal[3] = 1.91
        self.group_variable_values_arm_goal[4] = 0.0
        self.execute_arm_cmds()

    def dump_object(self):
        """
        Dump object pose
        """
        self.group_variable_values_arm_goal[0] = 0.0
        self.group_variable_values_arm_goal[1] = 1.52
        self.group_variable_values_arm_goal[2] = 0.0
        self.group_variable_values_arm_goal[3] = 0.0
        self.group_variable_values_arm_goal[4] = 0.0
        self.execute_arm_cmds()

    def main(self):
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

        rospy.loginfo("Shutting Down ..")
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node("joint_cmds_node", anonymous=True)
    pick_place_object = JointCommands()
    try:
        pick_place_object.main()
    except rospy.ROSInterruptException:
        pass
