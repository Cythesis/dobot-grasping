#!/usr/bin/env python
# license removed for brevity
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

def talker():
    pub = rospy.Publisher('ar_pose_marker', AlvarMarkers, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    	msg = AlvarMarkers();
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass