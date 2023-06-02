#! /usr/bin/env python
import rospy
import rosparam


class GoalPoses:
    def __init__(self):
        # Get the parameters from the ROS Param file: goal_poses.yaml

        self.position_x = rosparam.get_param("approach/position/x")
        self.position_y = rosparam.get_param("approach/position/y")
        self.position_z = rosparam.get_param("approach/position/z")
        self.orientation_x = rosparam.get_param("approach/orientation/x")
        self.orientation_y = rosparam.get_param("approach/orientation/y")
        self.orientation_z = rosparam.get_param("approach/orientation/z")
        self.orientation_w = rosparam.get_param("approach/orientation/w")

        self.position2_x = rosparam.get_param("release_position/position/x")
        self.position2_y = rosparam.get_param("release_position/position/y")
        self.position2_z = rosparam.get_param("release_position/position/z")
        self.orientation2_x = rosparam.get_param("release_position/orientation/x")
        self.orientation2_y = rosparam.get_param("release_position/orientation/y")
        self.orientation2_z = rosparam.get_param("release_position/orientation/z")
        self.orientation2_w = rosparam.get_param("release_position/orientation/w")


if __name__ == "__main__":
    rospy.init_node("goal_poses", anonymous=True)
    rb1_object = GoalPoses()
