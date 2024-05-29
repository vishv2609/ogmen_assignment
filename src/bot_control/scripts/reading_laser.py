#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

def msgCallback(msg):
    print("publishing filtered laser data")
    new_msg=Float64MultiArray()
    new_msg.data=msg.ranges[240:480]
    pub.publish(new_msg)
if __name__ == '__main__':
    rospy.init_node('filtering_node')
    rospy.Subscriber('/scan', LaserScan, msgCallback)
    pub=rospy.Publisher('/filtered_scan',Float64MultiArray,queue_size=10)
    rate=rospy.Rate(10)
    rate.sleep()
    rospy.spin()
