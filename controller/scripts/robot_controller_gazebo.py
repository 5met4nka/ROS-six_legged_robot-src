#!/usr/bin/env python3
#Author: lnotspotl

import rospy
import sys, time
from sensor_msgs.msg import Joy,Imu
from RobotController import RobotController
from InverseKinematics import robot_IK
from std_msgs.msg import Float64

USE_IMU = True
RATE = 60

rospy.init_node("Robot_Controller")

# Robot geometry
body = [0.09, 0.075, 0.085] #0, 1: fromt right 2: right side
legs = [0.045, 0.045, 0.009, 0.091] 

spot_robot = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics = robot_IK.InverseKinematics(body, legs)

command_topics = ["/spot_gazebo/FL_cntr/command",
                  "/spot_gazebo/FL_servo_cntr/command",
                  "/spot_gazebo/FL_link1_cntr/command",
                  "/spot_gazebo/FL_link2_cntr/command",
                  "/spot_gazebo/FR_cntr/command",
                  "/spot_gazebo/FR_servo_cntr/command",
                  "/spot_gazebo/FR_link1_cntr/command",
                  "/spot_gazebo/FR_link2_cntr/command",
                  "/spot_gazebo/ML_cntr/command",
                  "/spot_gazebo/ML_servo_cntr/command",
                  "/spot_gazebo/ML_link1_cntr/command",
                  "/spot_gazebo/ML_link2_cntr/command",
                  "/spot_gazebo/MR_cntr/command",
                  "/spot_gazebo/MR_servo_cntr/command",
                  "/spot_gazebo/MR_link1_cntr/command",
                  "/spot_gazebo/MR_link2_cntr/command",
                  "/spot_gazebo/RL_cntr/command",
                  "/spot_gazebo/RL_servo_cntr/command",
                  "/spot_gazebo/RL_link1_cntr/command",
                  "/spot_gazebo/RL_link2_cntr/command",
                  "/spot_gazebo/RR_cntr/command",
                  "/spot_gazebo/RR_servo_cntr/command",
                  "/spot_gazebo/RR_link1_cntr/command",
                  "/spot_gazebo/RR_link2_cntr/command"]

publishers = []
print(f"len(command_topics) : {len(command_topics)}")
for i in range(len(command_topics)):
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 0))

if USE_IMU:
    rospy.Subscriber("spot_imu/base_link_orientation",Imu,spot_robot.imu_orientation)
rospy.Subscriber("slr_joy/joy_ramped",Joy,spot_robot.joystick_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
del RATE

#time.sleep(10)

while not rospy.is_shutdown():
    leg_positions = spot_robot.run()
    spot_robot.change_controller()

    dx = spot_robot.state.body_local_position[0]
    dy = spot_robot.state.body_local_position[1]
    dz = spot_robot.state.body_local_position[2]
    
    roll = spot_robot.state.body_local_orientation[0]
    pitch = spot_robot.state.body_local_orientation[1]
    yaw = spot_robot.state.body_local_orientation[2]

    try:
        joint_angles = inverseKinematics.inverse_kinematics(leg_positions,
                               dx, dy, dz, roll, pitch, yaw)

        for i in range(len(joint_angles)):
            publishers[i].publish(joint_angles[i])
             
    except:
        import traceback
        traceback.print_exc(file=sys.stdout)
        rospy.loginfo(f"Can not solve inverese kinematics")

    rate.sleep()
