#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

def perform_trajectory():
    rospy.init_node('robot_trajectory_controller',anonymous=False)
    controller_name = "/primary_mnipulator_controller/command"
    pub = rospy.Publisher(controller_name, JointTrajectory, queue_size=10)
    argv = sys.argv[1:]
    robot_joints = ['joint_base', 'joint_link_1', 'joint_link_2', 'joint_link_3', 'joint_link_4', 'joint_gripper']
    goal_pos = [float(argv[0]), float(argv[1]), float(argv[2]), float(argv[3]), float(argv[4]), float(argv[5])]

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
    perform_trajectory()