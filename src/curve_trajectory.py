#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import numpy as np
import math
from spatialmath.base import *
from spatialmath import SE3
import spatialmath.base.symbolic as sym
import roboticstoolbox as rtb


def perform_trajectory(q):
    rospy.init_node('robot_trajectory_controller',anonymous=False)
    controller_name = "/primary_mnipulator_controller/command"
    pub = rospy.Publisher(controller_name, JointTrajectory, queue_size=10)
    robot_joints = ['joint_base', 'joint_link_1', 'joint_link_2', 'joint_link_3', 'joint_link_4', 'joint_gripper', 'joint_gripper_left','joint_gripper_right']
    gripper = np.array([0.0, 0.0])
    print("goal")
    goal_pos = np.append(q, gripper)
    print('output', goal_pos)
   

    rospy.loginfo('Goal position is set!')
    rospy.sleep(1)

    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = robot_joints
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = goal_pos
    trajectory_msg.points[0].velocities = [0.0 for i in robot_joints]
    trajectory_msg.points[0].accelerations = [0.0 for i in robot_joints]
    trajectory_msg.points[0].time_from_start = rospy.Duration(3)
    rospy.sleep(1)
    pub.publish(trajectory_msg)

if __name__ == '__main__':

    Link_1 = rtb.DHLink(165.5, -np.pi/2, 0, 85.5, qlim=[-1.571, 1.571])
    Link_2 = rtb.DHLink(0, 0, 0, 200, qlim=[-2.42, 0.71])
    Link_3 = rtb.DHLink(20.45, np.pi/2, 0, 0, qlim=[-1.0, 3.14])
    Link_4 = rtb.DHLink(147.93, -np.pi/2, 0, 0, qlim=[-1.571, 1.571])
    Link_5 = rtb.DHLink(0, np.pi/2, 0, qlim=[-1.571, 1.571])
    Link_6 = rtb.DHLink(179.51, 0, 0, 0, qlim=[-1.571, 1.571])
    custom_robot = rtb.DHRobot([Link_1, Link_2, Link_3, Link_4, Link_5, Link_6])
    print(custom_robot)


    waypoints = [
        SE3([200.5, 20.45, 300]),
        # SE3([250, 20.45, 300]),
        SE3([300, 20.45, 300])
    ]
    for i in waypoints:
        desired_position =  i
        print(i)
        q = custom_robot.ikine_LM(desired_position)
        print(custom_robot.fkine(q.q))
        print(q.q)
        perform_trajectory(q.q)