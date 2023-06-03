#! /usr/bin/env python
import rospy
import rosparam


class GoalPoses:
    def __init__(self):
        # Get the parameters from the ROS Param file: goal_poses.yaml

        self.position_x = rosparam.get_param("approach_coke_can/position/x")
        self.position_y = rosparam.get_param("approach_coke_can/position/y")
        self.position_z = rosparam.get_param("approach_coke_can/position/z")
        self.orientation_x = rosparam.get_param("approach_coke_can/orientation/x")
        self.orientation_y = rosparam.get_param("approach_coke_can/orientation/y")
        self.orientation_z = rosparam.get_param("approach_coke_can/orientation/z")
        self.orientation_w = rosparam.get_param("approach_coke_can/orientation/w")

        self.position2_x = rosparam.get_param("release_coke_can/position/x")
        self.position2_y = rosparam.get_param("release_coke_can/position/y")
        self.position2_z = rosparam.get_param("release_coke_can/position/z")
        self.orientation2_x = rosparam.get_param("release_coke_can/orientation/x")
        self.orientation2_y = rosparam.get_param("release_coke_can/orientation/y")
        self.orientation2_z = rosparam.get_param("release_coke_can/orientation/z")
        self.orientation2_w = rosparam.get_param("release_coke_can/orientation/w")


if __name__ == "__main__":
    rospy.init_node("goal_poses", anonymous=True)
    rb1_object = GoalPoses()
