#! /usr/bin/env python

import sys
import rospy
import moveit_commander

from geometry_msgs.msg import Pose
from joint_cmds import Joint_Cmds


class Pick_Perception:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.sub = rospy.Subscriber("/graspable_object_pose", Pose, self.pose_callback)
        self.pose_target = Pose()
        self.pick_joint_cmds = Joint_Cmds()

    def pose_callback(self, msg):
        # This is the pose of the graspable object from the robot_footprint frame
        self.pose_x_from_cam = msg.position.x
        self.pose_y_from_cam = msg.position.y
        self.pose_z_from_cam = msg.position.z

    def get_data(self):
        """
        Get useful data from MoveIt
        """
        self.pick_joint_cmds.group_arm.allow_replanning(True)
        self.pick_joint_cmds.group_arm.set_planning_time(15)
        self.pick_joint_cmds.group_arm.set_goal_position_tolerance(0.01)
        self.pick_joint_cmds.group_arm.set_goal_orientation_tolerance(0.05)

        print(
            "Reference frame: %s" % self.pick_joint_cmds.group_arm.get_planning_frame()
        )

        print(
            "End effector: %s" % self.pick_joint_cmds.group_arm.get_end_effector_link()
        )

        print("Robot Groups:")
        print(self.pick_joint_cmds.robot.get_group_names())

        print("Current Joint Values:")
        print(self.pick_joint_cmds.group_arm.get_current_joint_values())

        print("Current Pose:")
        print(self.pick_joint_cmds.group_arm.get_current_pose())

        print("Robot State:")
        print(self.pick_joint_cmds.robot.get_current_state())

    def pregrasp(self):
        """
        Pregrasp using manipulation/perception pipeline
        """
        offset_x = 0.005
        offset_y = 0.007
        offset_z = 0.035

        self.pose_target.position.x = self.pose_x_from_cam + offset_x
        self.pose_target.position.y = self.pose_y_from_cam + offset_y
        self.pose_target.position.z = self.pose_z_from_cam + offset_z
        self.pose_target.orientation.w = 1

        # Print our goal pose
        rospy.loginfo(
            "GOAL POSE FROM THE BASE_LINK:\t (x, y, z) = (%.2f, %.2f, %.2f)"
            % (
                self.pose_target.position.x,
                self.pose_target.position.y,
                self.pose_target.position.z,
            ),
        )
        self.pick_joint_cmds.group_arm.set_pose_target(self.pose_target)
        self.pick_joint_cmds.group_arm.go(wait=True)
        rospy.sleep(2)

    def main(self):
        rospy.loginfo("Getting Data..")
        pick_place_object.get_data()
        rospy.loginfo("Open Gripper..")
        self.pick_joint_cmds.open_gripper()
        rospy.loginfo("Pregrasp..")
        pick_place_object.pregrasp()
        rospy.loginfo("Grasp the object..")
        self.pick_joint_cmds.grasp()
        rospy.loginfo("Retreating..")
        self.pick_joint_cmds.retreat()

        rospy.loginfo("Shuting Down ..")
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node("pick_perception_node", anonymous=True)
    pick_place_object = Pick_Perception()
    try:
        pick_place_object.main()
    except rospy.ROSInterruptException:
        pass
