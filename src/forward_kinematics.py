import numpy as np
import math
from spatialmath.base import *
from spatialmath import SE3
import spatialmath.base.symbolic as sym
import roboticstoolbox as rtb
from sympy import Matrix
from scipy.spatial.transform import Rotation as R

def SE3(position, roll, pitch, yaw):
    # Assuming you construct the transformation matrix here
    # Using roll, pitch, and yaw to generate rotation matrix
    # Combining translation and rotation to create the SE(3) transformation matrix
    # This code depends on how your SE(3) function is implemented

    # Example: Constructing a simple SE(3) transformation matrix
    rotation_matrix = np.array([
        [np.cos(yaw)*np.cos(pitch), -np.sin(yaw)*np.cos(roll) + np.cos(yaw)*np.sin(pitch)*np.sin(roll), np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.sin(pitch)*np.cos(roll)],
        [np.sin(yaw)*np.cos(pitch), np.cos(yaw)*np.cos(roll) + np.sin(yaw)*np.sin(pitch)*np.sin(roll), -np.cos(yaw)*np.sin(roll) + np.sin(yaw)*np.sin(pitch)*np.cos(roll)],
        [-np.sin(pitch), np.cos(pitch)*np.sin(roll), np.cos(pitch)*np.cos(roll)]
    ])

    # Constructing the SE(3) transformation matrix by combining translation and rotation
    se3_matrix = np.eye(4)
    se3_matrix[:3, :3] = rotation_matrix
    se3_matrix[:3, 3] = position
    return se3_matrix

Link_1 = rtb.DHLink(165.5, -np.pi/2, 0, 85.5, qlim=[-1.571, 1.571])
Link_2 = rtb.DHLink(0, 0, 0, 200, qlim=[-2.42, 0.71])
Link_3 = rtb.DHLink(20.45, np.pi/2, 0, 0, qlim=[-1.0, 3.14])
Link_4 = rtb.DHLink(147.93, -np.pi/2, 0, 0, qlim=[-1.571, 1.571])
Link_5 = rtb.DHLink(0, np.pi/2, 0, qlim=[-1.571, 1.571])
Link_6 = rtb.DHLink(179.51, 0, 0, 0, qlim=[-1.571, 1.571])

custom_robot = rtb.DHRobot([Link_1, Link_2, Link_3, Link_4, Link_5, Link_6])
print(custom_robot)

q1 = 0
q2 = 0
q3 = 0
q4 = 0
q5 = 0
q6 = 0
T = custom_robot.fkine([math.radians(q1), math.radians(q2), math.radians(q3), math.radians(q4), math.radians(q5), math.radians(q6)])
print('Transformation Matrix :\n', T)

# desired_position = SE3([200.5, 20.45, 300]) 
desired_position = np.array([612, 20.45, 165])
roll = 90
pitch = 90
yaw = 90




# Create the SE(3) transformation matrix
transform_matrix = SE3(desired_position, roll, pitch, yaw)
q = custom_robot.ikine_LM(transform_matrix)

# rotvec = np.array([roll, pitch, yaw])
# rot = R.from_euler('xyz', rotvec)
# desired_orientation = rot.as_matrix()
# q = custom_robot.ikine_LM(desired_position)
T = custom_robot.fkine(q.q)
print(q)
print(T)