#!/usr/bin/env python3

import rospy
from mav_msgs.msg import RollPitchYawrateThrust
from trajectory_msgs.msg import MultiDOFJointTrajectory
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import numpy as np
import math
from simple_pid import PID
from scipy.spatial.transform import Rotation

# Constants
K_POS = np.array([1.0, 1.0, 1.0])
K_VEL = np.array([1.0, 1.0, 1.0])

# Tracking waypoints
current_waypoint = 0
waypoints = []

# Init PIDs, BS values
roll_pid = PID(1.0, 0.0, 0.0, setpoint=0)
pitch_pid = PID(1.0, 0.0, 0.0, setpoint=0)
yaw_rate_pid = PID(1.0, 0.0, 0.0, setpoint=0)

def calculate_acceleration_des(p, p_ref, v):
    a_fb = -K_POS*(p - p_ref) + -K_VEL*v
    a_des = a_fb + np.array([0.0, 0.0, 9.81])
    
    return a_des

def calculate_orientation_des(yaw, a_des):
    x_c = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    y_c = np.array([-math.sin(yaw), math.cos(yaw), 0.0])

    z_B_des = a_des / np.linalg.norm(a_des)
    x_B_des = np.cross(y_c, z_B_des) / np.linalg.norm(np.cross(y_c, z_B_des))
    y_B_des = np.cross(z_B_des, x_B_des)
    R_des = np.array([x_B_des, y_B_des, z_B_des])

    # Convert to quaternion
    r = Rotation.from_matrix(R_des)
    q_des = r.as_quat()

    return q_des

def odometry_callback(data_msg):
    global current_waypoint, waypoints

    position = data_msg.pose.pose.position
    velocity = data_msg.twist.twist.linear
    orientation = data_msg.pose.pose.orientation
    yaw = orientation.z


    if current_waypoint < len(waypoints):
        position_des = waypoints[current_waypoint].transforms[0].translation
        desired_yaw = math.atan2(position_des.y - position.y, position_des.x - position.x)
    else:
        position_des = position
        desired_yaw = yaw

    # Controller calculations from suggested paper
    a_des = calculate_acceleration_des(
        np.array([position.x, position.y, position.z]), 
        np.array([position_des.x, position_des.y, position_des.z]), 
        np.array([velocity.x, velocity.y, velocity.z]))
    
    orientation_des = calculate_orientation_des(yaw, a_des)

    # PID
    roll_cmd = roll_pid(orientation_des[0] - orientation.x)
    pitch_cmd = pitch_pid(orientation_des[1] - orientation.y)
    yaw_rate_cmd = yaw_rate_pid(orientation_des[2] - orientation.z)

    # Publish message
    cmd_msg = RollPitchYawrateThrust()
    cmd_msg.header.stamp = rospy.Time.now()
    cmd_msg.roll = roll_cmd
    cmd_msg.pitch = pitch_cmd
    cmd_msg.yaw_rate = yaw_rate_cmd
    cmd_msg.thrust.z = a_des[2]*0.5

    pub.publish(cmd_msg)

    # Increment waypoint if close enough
    # distance_to_waypoint = math.sqrt((position.x - position_des.x)**2 + (position.y - position_des.y)**2)
    # if distance_to_waypoint < 0.5:
    #     current_waypoint += 1

def trajectory_callback(data_msg):
    global waypoints, current_waypoint
    waypoints = data_msg.points
    current_waypoint = 0

if __name__ == '__main__':
    rospy.init_node("PID_position_yaw_controller_node", anonymous=True)
    pub = rospy.Publisher('rmf_obelix/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=10)
    trajectory_sub = rospy.Subscriber('rmf_obelix/command/trajectory', MultiDOFJointTrajectory, trajectory_callback)
    odometry_sub = rospy.Subscriber('rmf_obelix/odometry_sensor1/odometry', Odometry, odometry_callback)
    rospy.loginfo("TTK22 Project 1 node started")

    rospy.spin()