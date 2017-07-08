#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
import time
import random


joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def callback1(data,x):
	cmd_publisher = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
	for i in range(len(data.buttons)):
		if data.buttons[i] == 1:
			break
	cmd = JointTrajectory()
	cmd.header.stamp = rospy.Time.now()
	cmd.joint_names = joint_names
	points = []
	positions = list(x.position)
	point = JointTrajectoryPoint()
	factor = 1
	if i >= 10 and i <16:
		#print(data.axes)
		#print(positions)
		if i == 12:
			factor = -1
		positions[i-10] = data.axes[2] * factor
		point.positions = positions
		point.time_from_start.secs = 0.0
		point.time_from_start.nsecs = 157114594
		points.append(point)
		cmd.points = points
		cmd_publisher.publish(cmd)

def callback(data):
	joy_subscriber = rospy.Subscriber('/joy',Joy,callback1,data)
	rospy.spin()

def init():
    rospy.init_node('safety_controller')
    joint_state_subscriber = rospy.Subscriber('/joint_states',JointState,callback)
    rospy.spin()
    print 'Sent message!'



if __name__ == '__main__':
    init()