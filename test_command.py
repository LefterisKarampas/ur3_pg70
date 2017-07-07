#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time
import random

def init():
    rospy.init_node('safety_controller')
    cmd_publisher = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10);
    cmd = JointTrajectory()
    cmd.header.stamp = rospy.Time.now()
    joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    #joint_names.append('wrist_2_joint')
    cmd.joint_names = joint_names
    points = []
    point = JointTrajectoryPoint()
    rand = random.uniform(0.5, 1.0)
    positions = [-0.367360865402936, -0.46427831689094634, 0.0009054034303047523, -1.0682320113974995, -0.00012542263742842372, 1.9039493380454027]
    for i in range(len(positions)):
        positions[i] *= rand
    point.positions = positions
    point.time_from_start.secs = 0
    point.time_from_start.nsecs = 157114594
    points.append(point)
    cmd.points = points


    time.sleep(5)
    cmd_publisher.publish(cmd)
    print 'Sent message!'



if __name__ == '__main__':
    init()