#! /usr/bin/env python
"""Node ur5_1"""
import rospy
import sys
import copy
import threading
import cv2
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import numpy as np
import rospkg
import yaml
import time

from std_msgs.msg import String
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg


class Ur5Moveit(object):

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._robot_ns = '/'  + 'ur5_1'
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        """This method ensures collision updates are recieved"""

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self._scene.get_attached_objects([self._box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            is_known = self._box_name in self._scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
    

    def attach_box(self, name):
        """This method includes the box in planning scene to avoid collisions in path planning"""
        timeout=4
        self._box_name = name
        grasping_group = 'manipulator'
        touch_links = self._robot.get_link_names(group=grasping_group)
        self._scene.attach_box(self._eef_link, self._box_name, touch_links=touch_links)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def remove_box(self, timeout=4):
        """This method removes the box from the planning scene"""

        self._scene.remove_world_object(self._box_name)
        return self.wait_for_state_update(box_is_attached=False,
                                          box_is_known=False,
                                          timeout=timeout)
                                        
    def detach_box(self, timeout=4):
        """This method detaches the box in rviz"""
        self._scene.remove_attached_object(self._eef_link, name=self._box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


    def clear_octomap(self):
		clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
		return clear_octomap_service_proxy()


    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    
    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        ret = self._group.execute(loaded_plan)
        return ret


    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False
        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
        
        return True



    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def gripperClient(flag):
        """Client to activate or deactivate the gripper"""

        print"Waiting for service.."
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        print"Done."
        try:
            resp1 = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
            resp1(flag)
        except rospy.ServiceException as e:
            print "Service call failed : %s"%e


def main():

    for i in range(30):
        print"Waiting 30 secs for packages to load pls dont interrupt!"
        rospy.sleep(1)

    ur5 = Ur5Moveit()

    k=0

    lst_joint_angles_0 = [0.12225186436517266,
                          -2.338769815082638,
                          -1.0994678869384105,
                          -1.2217393040321394,
                           1.5533070014238675,
                          0.1397012056572393]

    while not rospy.is_shutdown():
        rospy.logwarn("\n\nPose#0")
        ur5.hard_set_joint_angles(lst_joint_angles_0, 50)
        rospy.sleep(k)

        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '0to1.yaml', 5)
        gripperClient(True)
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '1to0.yaml', 5)
        gripperClient(False)

        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '0to2.yaml', 5)
        gripperClient(True)
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '2to0.yaml', 5)
        gripperClient(False)

        rospy.logwarn("1. Playing AllZeros to P3245345ose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '0to3.yaml', 5)
        gripperClient(True)
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '3to0.yaml', 5)
        gripperClient(False)

        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '0to4.yaml', 5)
        gripperClient(True)
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '4to0.yaml', 5)
        gripperClient(False)

        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '0to5.yaml', 5)
        gripperClient(True)
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '5to0.yaml', 5)
        gripperClient(False)

        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '0to6.yaml', 5)
        gripperClient(True)
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '6to0.yaml', 5)
        gripperClient(False)

        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '0to7.yaml', 5)
        gripperClient(True)
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '7to0.yaml', 5)
        gripperClient(False)

        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '0to8.yaml', 5)
        gripperClient(True)
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '8to0.yaml', 5)
        gripperClient(False)

        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '0to9.yaml', 5)
        gripperClient(True)
        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '9to0.yaml', 5)
        gripperClient(False)

        break



    del ur5



if __name__ == '__main__':
    main()