#! /usr/bin/env python

import sys
import rospy
import moveit_commander

from geometry_msgs.msg import Pose
from joint_cmds import JointCommands


class PickPerception:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.sub = rospy.Subscriber("/graspable_object_pose", Pose, self.pose_callback)
        self.pose_target = Pose()
        self.joint_cmds = JointCommands()

    def pose_callback(self, msg):
        # This is the pose of the graspable object from the robot_footprint frame
        self.pose_x_from_cam = msg.position.x
        self.pose_y_from_cam = msg.position.y
        self.pose_z_from_cam = msg.position.z

    def _pregrasp_perception(self):
        # Pregrasp using manipulation/perception pipeline

        offset_x = 0.005
        offset_y = 0.007
        offset_z = 0.035

        self.pose_target.position.x = self.pose_x_from_cam + offset_x
        self.pose_target.position.y = self.pose_y_from_cam + offset_y
        self.pose_target.position.z = self.pose_z_from_cam + offset_z
        self.pose_target.orientation.w = 1

        rospy.loginfo(
            "GOAL POSE FROM THE BASE_LINK: (x, y, z) = (%.3f, %.3f, %.3f)"
            % (
                self.pose_target.position.x,
                self.pose_target.position.y,
                self.pose_target.position.z,
            ),
        )
        self.joint_cmds.group_arm.set_pose_target(self.pose_target)
        self.joint_cmds.group_arm.go(wait=True)
        rospy.sleep(2)

    def main(self):
        rospy.loginfo(
            "Reference frame to set end effector goal poses is: %s",
            self.joint_cmds.group_arm.get_pose_reference_frame(),
        )
        rospy.loginfo("Getting Data..")
        self.joint_cmds.set_group_config(15, 0.01, 0.05)
        rospy.loginfo("Open Gripper..")
        self.joint_cmds.open_gripper()
        rospy.loginfo("Pregrasp..")
        pick_place_object._pregrasp_perception()
        rospy.loginfo("Grasp the object..")
        self.joint_cmds.grasp()
        rospy.loginfo("Retreating..")
        self.joint_cmds.retreat()

        rospy.loginfo("Shutting Down ..")
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node("pick_perception_node", anonymous=True)
    pick_place_object = PickPerception()
    try:
        pick_place_object.main()
    except rospy.ROSInterruptException:
        pass
