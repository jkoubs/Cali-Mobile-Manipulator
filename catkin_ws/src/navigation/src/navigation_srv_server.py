#! /usr/bin/env python
import rospy
import actionlib
from navigation.srv import GoToPoi, GoToPoiResponse
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseResult,
    MoveBaseFeedback,
)

from goal_poses import GoalPoses


class GoToPOI:
    def __init__(self):
        self.srv_server = rospy.Service("/go_to_point", GoToPoi, self.main_callback)
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        # waits until the action server is up and running
        self.client.wait_for_server()
        self.goal_poses = GoalPoses()
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        """
        This works better than the rospy.is_shutdown()
        """
        self.ctrl_c = True

    def feedback_callback(self, feedback):
        """
        Feedback from Action Server
        """
        print("[Feedback] Going to Point of Interest...")

    def main_callback(self, request):
        """
        Navigation Action Server Callback
        """
        goal = MoveBaseGoal()

        if request.label == "approach":
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = self.goal_poses.position_x
            goal.target_pose.pose.position.y = self.goal_poses.position_y
            goal.target_pose.pose.position.z = self.goal_poses.position_z
            goal.target_pose.pose.orientation.x = self.goal_poses.orientation_x
            goal.target_pose.pose.orientation.y = self.goal_poses.orientation_y
            goal.target_pose.pose.orientation.z = self.goal_poses.orientation_z
            goal.target_pose.pose.orientation.w = self.goal_poses.orientation_w

        elif request.label == "release_position":
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = self.goal_poses.position2_x
            goal.target_pose.pose.position.y = self.goal_poses.position2_y
            goal.target_pose.pose.position.z = self.goal_poses.position2_z
            goal.target_pose.pose.orientation.x = self.goal_poses.orientation2_x
            goal.target_pose.pose.orientation.y = self.goal_poses.orientation2_y
            goal.target_pose.pose.orientation.z = self.goal_poses.orientation2_z
            goal.target_pose.pose.orientation.w = self.goal_poses.orientation2_w

        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.client.wait_for_result()

        print("[Result] State: %d" % (self.client.get_state()))

        response = GoToPoiResponse()
        response.success = "OK, Service Finished correctly"
        return response


if __name__ == "__main__":
    rospy.init_node("go_to_point_of_interest", anonymous=True)
    rb1_object = GoToPOI()
    rospy.spin()
