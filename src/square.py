#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from spatialmath import SE3
import spatialmath.base.symbolic as sym
import roboticstoolbox as rtb

def action_interface():
    rospy.init_node('dual_dual_arm_trajectorymsg_actionlib')
    robot_joints = ['joint_base', 'joint_link_1', 'joint_link_2', 'joint_link_3', 'joint_link_4', 'joint_gripper']
    waypoints_square = [
        [250, 40, 300],
        [250, 100, 300],
        [350 , 100, 300],
        [350, 40, 300]
    ]

    joints_trajectory_points = []
    Link_1 = rtb.DHLink(165.5, -np.pi/2, 0, 85.5, qlim=[-1.571, 1.571])
    Link_2 = rtb.DHLink(0, 0, 0, 200, qlim=[-2.42, 0.71])
    Link_3 = rtb.DHLink(20.45, np.pi/2, 0, 0, qlim=[-1.0, 3.14])
    Link_4 = rtb.DHLink(147.93, -np.pi/2, 0, 0, qlim=[-1.571, 1.571])
    Link_5 = rtb.DHLink(0, np.pi/2, 0, qlim=[-1.571, 1.571])
    Link_6 = rtb.DHLink(179.51, 0, 0, 0, qlim=[-1.571, 1.571])
    custom_robot = rtb.DHRobot([Link_1, Link_2, Link_3, Link_4, Link_5, Link_6])
    for i in range(4):
        point = SE3(waypoints_square[i])
        joints_trajectory_points.append(np.array((custom_robot.ikine_LM(point)).q))
    rospy.loginfo('inverse kinematics solved')
    robot_client = actionlib.SimpleActionClient('/primary_mnipulator_controller/follow_joint_trajectory/', FollowJointTrajectoryAction)
    robot_client.wait_for_server()
    rospy.loginfo('server connection established')
    rospy.sleep(1)
    for i in range (4):
        trajectory_message = JointTrajectory()
        trajectory_message.joint_names = robot_joints
        trajectory_message.points.append(JointTrajectoryPoint())
        trajectory_message.points[0].positions = joints_trajectory_points[i]
        trajectory_message.points[0].velocities = [0.0 for i in robot_joints]
        trajectory_message.points[0].accelerations = [0.0 for i in robot_joints]
        trajectory_message.points[0].time_from_start = rospy.Duration(3)
        goal_pos = FollowJointTrajectoryGoal()
        goal_pos.trajectory = trajectory_message
        goal_pos.goal_time_tolerance = rospy.Duration(0)
        robot_client.send_goal(goal_pos)
        rospy.sleep(1)


if __name__ == '__main__':
    action_interface()