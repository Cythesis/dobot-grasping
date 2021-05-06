#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String

def callback(data):
	msg = "Hello world"
	pub.publish(msg)

if __name__=='__main__':
	rospy.init_node('ar_pose_converted')
	rospy.Subscriber("/ar_pose_marker",AlvarMarkers,callback)
	pub = rospy.Publisher('ar_pose_converted', String, queue_size=1)
	rate=rospy.Rate(10)
	rospy.spin()

