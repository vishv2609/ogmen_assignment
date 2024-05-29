#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion

class ToGoal():

    def __init__(self):
        rospy.init_node('MoveRobot', anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.callback)

        self.goalx = 3.0
        self.goaly = 4.0

        # Initialize position and orientation
        self.positionx = 0.0
        self.positiony = 0.0
        self.yaw = 0.0

        self.rate = rospy.Rate(10)

    def callback(self, msg):
        """Callback function which is called when a new message is received by the subscriber."""
        self.positionx = msg.pose.pose.position.x
        self.positiony = msg.pose.pose.position.y
        self.orientation_q = msg.pose.pose.orientation
        self.orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(self.orientation_list)
        print((self.positionx, self.positiony, self.yaw))

    def euclidean_distance(self):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((self.goalx - self.positionx), 2) + pow((self.goaly - self.positiony), 2))

    def linear_vel(self):
        return 1.5 * self.euclidean_distance()

    def steering_angle(self):
        return atan2(self.goaly - self.positiony, self.goalx - self.positionx)

    def angular_vel(self):
        return 6 * (self.steering_angle() - self.yaw)

    def move2goal(self):
        """Moves the robot to the goal."""
        move = Twist()

        print("Distance to goal is %f" % self.euclidean_distance())

        while self.euclidean_distance() >= 0.01:
            print("Distance to goal is %f" % self.euclidean_distance())
            # Linear velocity in the x-axis.
            move.linear.x = 0.05*self.linear_vel()
            move.linear.y = 0
            move.linear.z = 0

            # Angular velocity in the z-axis.
            move.angular.x = 0
            move.angular.y = 0
            move.angular.z = 0.1*self.angular_vel()

            # Publishing our vel_msg
            self.pub.publish(move)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        move.linear.x = 0
        move.angular.z = 0
        self.pub.publish(move)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        objectToGoal = ToGoal()
        objectToGoal.move2goal()
    except rospy.ROSInterruptException:
        pass
