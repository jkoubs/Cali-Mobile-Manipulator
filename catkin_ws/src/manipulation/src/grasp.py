#! /usr/bin/env python

    # <arg name="x" default="3.0282"/>
    # <arg name="y" default="10.1044"/>
    # <arg name="z" default="0.1"/>
    # <arg name="roll" default="0"/>
	# <arg name="pitch" default="0"/>
	# <arg name="yaw" default="1.570796"/>

import sys
import copy
import tf
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist



class Pick_Place_EE_Pose():

    def __init__(self):

         ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints. 
        self.group_arm = moveit_commander.MoveGroupCommander("arm")
        self.group_gripper = moveit_commander.MoveGroupCommander("gripper")

        
        self.rb1_vel_publisher = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=1)

        self.sub = rospy.Subscriber(
            '/graspable_object_pose', Pose, self.pose_callback)

        # Get arm and gripper joint values
        self.group_variable_values_arm_goal = self.group_arm.get_current_joint_values()
        self.group_variable_values_gripper_close = self.group_gripper.get_current_joint_values()
        self.pose_target = Pose()
        self.reach = Pose()
        self.rate = rospy.Rate(10)

        self.offset_x = -0.04
        self.offset_y = 0.015
        self.offset_z = 0.033
        self.ctrl_c = False
        self.cmd = Twist()

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.rb1_vel_publisher.get_num_connections()
            if connections > 0:
                self.rb1_vel_publisher.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def pose_callback(self, msg):
        # This is the pose given from the camera 
        # From the robot_footprint to the graspable object
        self.pose_x_from_cam = msg.position.x
        self.pose_y_from_cam = msg.position.y
        self.pose_z_from_cam = msg.position.z

    def move_forward(self):
        rospy.loginfo("Moving forward !")
        self.cmd.linear.x = 0.01
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def stop(self):
        rospy.loginfo('Stop the robot !')
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def check_reach(self):
        self.reach.position.x = self.pose_x_from_cam
        while self.reach.position.x > 0.69:
            print(self.reach.position.x)
            self.move_forward()
            self.rate.sleep()
        self.stop()

    def get_data(self):

        self.group_arm.allow_replanning(True)
        self.group_arm.set_planning_time(30)
        self.group_arm.set_goal_position_tolerance(0.005)
        self.group_arm.set_goal_orientation_tolerance(0.05)
        # self.group_arm.set_goal_tolerance(0.01)
    
        print("Reference frame: %s" %
      self.group_arm.get_planning_frame())  # = world = base_link

        print("End effector: %s" % self.group_arm.get_end_effector_link())  # = end_effector_link

        print("Robot Groups:")
        print(self.robot.get_group_names())

        print("Current Joint Values:")
        print(self.group_arm.get_current_joint_values())

        print("Current Pose:")
        print(self.group_arm.get_current_pose())

        print("Robot State:")
        print(self.robot.get_current_state())
    
    
    def open_gripper(self):
        # Step1: Open Gripper joint value [GRIPPER GROUP]

        self.group_variable_values_gripper_close[0] = 1.57
        self.group_gripper.set_joint_value_target(
            self.group_variable_values_gripper_close)

        self.plan1 = self.group_gripper.plan()

        self.group_gripper.go(wait=True)
        rospy.sleep(2)

    def pregrasp(self):
        self.move_forward()
        while not self.pose_x_from_cam < 0.71:
            rospy.loginfo("<<<<<<<<<<<<<<<<<  OPTIMIZATION STEP : Slowly approaching the coke can  >>>>>>>>>>>>>>>>>>>>")
            rospy.loginfo("The distance to the coke can is: %f",self.pose_x_from_cam)
            self.rate.sleep()
        self.stop()
        print('reach =')
        print(self.pose_x_from_cam)

        self.pose_target.position.x = self.pose_x_from_cam + self.offset_x
        self.pose_target.position.y = self.pose_y_from_cam + self.offset_y
        self.pose_target.position.z = self.pose_z_from_cam + self.offset_z

        self.pose_target.orientation.x = 0.0
        self.pose_target.orientation.y = 0.0
        self.pose_target.orientation.z = 0.0
        self.pose_target.orientation.w = 1

        # Print our goal pose
        rospy.logerr("LETS PRINT OUR GOAL POSE FROM THE robot_footprint FRAME:")
        rospy.logerr(self.pose_target.position.x)
        rospy.logerr(self.pose_target.position.y)
        rospy.logerr(self.pose_target.position.z)
        print("POSITION:\n")
        print(self.pose_target.position.x)
        print(self.pose_target.position.y)
        print(self.pose_target.position.z)
        print("\nORIENTATION:\n")
        print(self.pose_target.orientation.x) 
        print(self.pose_target.orientation.y) 
        print(self.pose_target.orientation.z) 
        print(self.pose_target.orientation.w) 

        
        self.group_arm.set_pose_target(self.pose_target)
        self.plan2 = self.group_arm.plan()
        self.group_arm.go(wait=True)
        print("CONFIRMIIIIIIIIIIIING!!!!!!!!")
        rospy.logerr(self.pose_target.position.x)
        rospy.logerr(self.pose_target.position.y)
        rospy.logerr(self.pose_target.position.z)
        print("POSITION:\n")
        print(self.pose_target.position.x)
        print(self.pose_target.position.y)
        print(self.pose_target.position.z)
        print("\nORIENTATION:\n")
        print(self.pose_target.orientation.x) 
        print(self.pose_target.orientation.y) 
        print(self.pose_target.orientation.z) 
        print(self.pose_target.orientation.w) 
        rospy.sleep(2)

    def grasp(self):
        # Step3: Open Gripper joint value [GRIPPER GROUP]

        self.group_variable_values_gripper_close[0] = 0.6
        self.group_gripper.set_joint_value_target(
            self.group_variable_values_gripper_close)

        self.plan3 = self.group_gripper.plan()

        self.group_gripper.go(wait=True)
        rospy.sleep(2)

    def retreat(self):
        # Step4: Retreat/Move back [ARM GROUP]
        self.group_variable_values_arm_goal[0] = 0
        self.group_variable_values_arm_goal[1] = 0.45
        self.group_variable_values_arm_goal[2] = -0.82
        self.group_variable_values_arm_goal[3] = 1.95
        self.group_variable_values_arm_goal[4] = 0
        self.group_arm.set_joint_value_target(
            self.group_variable_values_arm_goal)

        self.plan4 = self.group_arm.plan()

        self.group_arm.go(wait=True)
        rospy.sleep(2)


    def main(self):
        
        rospy.loginfo('Getting Data..')
        pick_place_object.get_data()

        # rospy.loginfo('Now check if robot can reach the graspable object............')
        # pick_place_object.check_reach()

        # while self.pose_x_from_cam > 0.69:
        #     print(self.pose_x_from_cam)
        #     pick_place_object.move_forward()
        #     self.rate.sleep()
        # pick_place_object.stop()

        rospy.loginfo('Opening Gripper..')
        pick_place_object.open_gripper()
        rospy.loginfo('Pregrasp..')
        pick_place_object.pregrasp()
        rospy.loginfo('Grasp..')
        pick_place_object.grasp()
        rospy.loginfo('Retreating..')
        pick_place_object.retreat()
        
        rospy.loginfo('Shuting Down ..')
        moveit_commander.roscpp_shutdown()
    

if __name__ == '__main__':
    rospy.init_node('grasp_coke_can', anonymous=True)
    pick_place_object = Pick_Place_EE_Pose()
    try:
        pick_place_object.main()
    except rospy.ROSInterruptException:
        pass


