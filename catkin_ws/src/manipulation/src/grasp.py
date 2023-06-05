#! /usr/bin/env python

import sys
import rospy
import moveit_commander

from joint_cmds import JointCommands

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


class Grasp:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.rb1_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/graspable_object_pose", Pose, self.pose_callback)
        self.joint_cmds = JointCommands()
        self.pose_target = Pose()
        self.cmd = Twist()
        self.rate = rospy.Rate(10)
        self.ctrl_c = False

    def pose_callback(self, msg):
        # This is the pose given from the robot_footprint frame to the graspable object
        self.pose_x_from_cam = msg.position.x
        self.pose_y_from_cam = msg.position.y
        self.pose_z_from_cam = msg.position.z

    def _publish_once_in_cmd_vel(self):
        # This is because publishing in topics sometimes fails the first time you publish.
        # In continuous publishing systems, this is no big deal, but in systems that publish only
        # once, it IS very important.

        while not self.ctrl_c:
            connections = self.rb1_vel_publisher.get_num_connections()
            if connections > 0:
                self.rb1_vel_publisher.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def _move_forward(self):
        rospy.loginfo("Moving forward..")
        self.cmd.linear.x = 0.01
        self.cmd.angular.z = 0.0
        self._publish_once_in_cmd_vel()

    def _stop(self):
        rospy.loginfo("Stop the robot..")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self._publish_once_in_cmd_vel()

    def _approach(self):
        self._move_forward()
        while not self.pose_x_from_cam < 0.71:
            rospy.loginfo(
                "APPROACH STEP : The distance to the coke can is: %f"
                % self.pose_x_from_cam
            )
            self.rate.sleep()
        self._stop()

    def _pregrasp(self):
        offset_x = -0.04
        offset_y = 0.015
        offset_z = 0.033

        self._approach()
        rospy.loginfo("Pregrasp..")
        self.pose_target.position.x = self.pose_x_from_cam + offset_x
        self.pose_target.position.y = self.pose_y_from_cam + offset_y
        self.pose_target.position.z = self.pose_z_from_cam + offset_z
        self.pose_target.orientation.w = 1

        # Print our goal pose
        rospy.loginfo(
            "LET'S PRINT OUR GOAL POSE FROM THE robot_footprint FRAME: (%.3f, %.3f, %.3f)"
            % (
                self.pose_target.position.x,
                self.pose_target.position.y,
                self.pose_target.position.z,
            )
        )
        self.joint_cmds.group_arm.set_pose_target(self.pose_target)
        self.joint_cmds.group_arm.go(wait=True)
        rospy.sleep(2)

    def main(self):
        rospy.loginfo("Getting Data..")
        self.joint_cmds.set_group_config(30, 0.005, 0.05)
        rospy.loginfo("Opening Gripper..")
        self.joint_cmds.open_gripper()
        pick_place_object._pregrasp()
        rospy.loginfo("Grasp..")
        self.joint_cmds.grasp()
        rospy.loginfo("Retreating..")
        self.joint_cmds.retreat()

        rospy.loginfo("Shutting Down ..")
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node("grasp_node", anonymous=True)
    pick_place_object = Grasp()
    try:
        pick_place_object.main()
    except rospy.ROSInterruptException:
        pass
