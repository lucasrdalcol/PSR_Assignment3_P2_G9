#!/usr/bin/env python3

import copy
import rospy
import math
import tf2_ros
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PoseStamped

class Driver:

    def __init__(self):

        # Define a goal Pose to which the robot should move
        self.goal = PoseStamped()
        self.goal_active = False   # The goal is active or not

        self.linear = 0
        self.angular = 0

        self.name = rospy.get_name()
        self.name = self.name.strip('/') # remove "/"
        # self.name = self.name.strip('_driver') # remove _driver
        print('My player name is ' + self.name)

        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.sendCommandCallback)  # Calls the function sendCommandCallback for each 0.1 sec
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalReceivedCallback)


    def goalReceivedCallback(self, msg):

        rospy.loginfo('Received new goal on frame id ' + msg.header.frame_id)
        target_frame = self.name + '/odom'

        try:
            self.goal = self.tf_buffer.transform(msg, target_frame, rospy.Duration(1))
            self.goal_active = True
            rospy.logwarn('Setting new goal')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.goal_active = False
            rospy.logerr('Could not transform goal from ' + msg.header.frame_id + ' to ' + target_frame + '. Will ignore this goal.')


    def driveStraight(self, minimum_speed = 0.1, maximum_speed = 0.6):

        goal_copy = copy.deepcopy(self.goal)
        goal_copy.header.stamp = rospy.Time.now()

        # print('Transforming pose')
        goal_in_base_link = self.tf_buffer.transform(goal_copy, self.name + '/base_footprint', rospy.Duration(1))
        # print('Pose trasnformed')

        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y

        self.angle = math.atan2(y,x)

        distance_to_goal = math.sqrt(x**2 + y**2)
        self.speed = max(minimum_speed, 0.3 * distance_to_goal)   # limit minimum speed
        self.speed = min(maximum_speed, self.speed)   # limit maximum speed

        if distance_to_goal < 0.03:
            rospy.logwarn('I have achieved my goal!')
            self.goal_active = False


    def sendCommandCallback(self, event):
        print('Sending twist command')

        # Decision outputs a speed (linear velocity) and angle (angular velocity)
        # input: goal
        # output: angle and speed

        if not self.goal_active:  # no goal, no movement
            self.angle = 0
            self.speed = 0
        else:
            self.driveStraight()

        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angle
        self.publisher_command.publish(twist)

def main():
    # ---------------------------------------------------
    # INITIALIZATION
    # ---------------------------------------------------

    rospy.init_node('p_vbatista_driver', anonymous=False)

    driver = Driver()
    rospy.spin()

if __name__ == '__main__':
    main()