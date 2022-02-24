#!/usr/bin/env python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
import copy
import math
import colorama
import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from psr_parte09_exs.msg import Dog
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PoseStamped
from sensor_msgs.msg import Image


class Driver:
    def __init__(self):
        # Define all variables
        self.name = rospy.get_name()
        self.name = self.name.strip('/')  # remove initial /
        self.goal = PoseStamped()
        self.goal_active = False
        self.prey_active = False
        self.angle = 0
        self.speed = 0

        # Initialize publisher
        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)

        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Use Timer to set a Command Callback
        self.timer = rospy.Timer(rospy.Duration(0.1), self.sendCommandCallback)

        # Initialize the subscriber to receive the goal pose with a callback
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped,
                                                self.goalReceivedCallback)  # Subscribe the node to the specified topic

        # Which is my team
        self.configureMyTeam(self.name)

        # Initialize the color segmentation
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber('/' + self.name + '/camera/rgb/image_raw', Image,
                                                 self.configureColorSegmentationCallback)

    def configureMyTeam(self, player_name):
        self.teams = {'red_team': rospy.get_param('/red_players'),
                      'green_team': rospy.get_param('/green_players'),
                      'blue_team': rospy.get_param('/blue_players')}

        if self.name in self.teams['red_team']:
            self.teams['my_team'], self.teams['my_preys'], self.teams[
                'my_hunters'] = 'red_team', 'green_team', 'blue_team'
            rospy.loginfo('My name is ' + self.name + '. I am team red, I am hunting ' + str(self.teams['green_team']) +
                          ' and I am fleeing from ' + str(self.teams['blue_team']))
        elif self.name in self.teams['green_team']:
            self.teams['my_team'], self.teams['my_preys'], self.teams[
                'my_hunters'] = 'green_team', 'blue_team', 'red_team'
            rospy.loginfo(
                'My name is ' + self.name + '. I am team green, I am hunting ' + str(self.teams['blue_team']) +
                ' and I am fleeing from ' + str(self.teams['red_team']))
        elif self.name in self.teams['blue_team']:
            self.teams['my_team'], self.teams['my_preys'], self.teams[
                'my_hunters'] = 'blue_team', 'red_team', 'green_team'
            rospy.loginfo('My name is ' + self.name + '. I am team blue, I am hunting ' + str(self.teams['red_team']) +
                          ' and I am fleeing from ' + str(self.teams['green_team']))
        else:
            raise rospy.logerr('The player name' + self.name + ' does not fit in any team.')

    def configureColorSegmentationCallback(self, image_msg):
        # Convert the from imgmsg ROS to opencv.
        try:
            rgb_image_original = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # Segment the color for red players
        ranges_red = {'b': {'min': 0, 'max': 50},
                      'g': {'min': 0, 'max': 50},
                      'r': {'min': 100, 'max': 256}}

        # Processing
        mins_red = np.array([ranges_red['b']['min'], ranges_red['g']['min'], ranges_red['r']['min']])
        maxs_red = np.array([ranges_red['b']['max'], ranges_red['g']['max'], ranges_red['r']['max']])
        mask_red = cv2.inRange(rgb_image_original, mins_red, maxs_red)
        # conversion from numpy from uint8 to bool
        mask_red = mask_red.astype(bool)

        # Segment the color for green players
        ranges_green = {'b': {'min': 0, 'max': 50},
                        'g': {'min': 100, 'max': 256},
                        'r': {'min': 0, 'max': 50}}

        # Processing
        mins_green = np.array([ranges_green['b']['min'], ranges_green['g']['min'], ranges_green['r']['min']])
        maxs_green = np.array([ranges_green['b']['max'], ranges_green['g']['max'], ranges_green['r']['max']])
        mask_green = cv2.inRange(rgb_image_original, mins_green, maxs_green)
        # conversion from numpy from uint8 to bool
        mask_green = mask_green.astype(bool)

        # Segment the color for blue players
        ranges_blue = {'b': {'min': 100, 'max': 256},
                       'g': {'min': 0, 'max': 50},
                       'r': {'min': 0, 'max': 50}}

        # Processing
        mins_blue = np.array([ranges_blue['b']['min'], ranges_blue['g']['min'], ranges_blue['r']['min']])
        maxs_blue = np.array([ranges_blue['b']['max'], ranges_blue['g']['max'], ranges_blue['r']['max']])
        mask_blue = cv2.inRange(rgb_image_original, mins_blue, maxs_blue)
        # conversion from numpy from uint8 to bool
        mask_blue = mask_blue.astype(bool)

        # Paint largest blobs
        rgb_image_processed = copy.deepcopy(rgb_image_original)
        rgb_image_processed = self.paintBlobs(rgb_image_processed, mask_red, color='red')
        rgb_image_processed = self.paintBlobs(rgb_image_processed, mask_green, color='green')
        rgb_image_processed = self.paintBlobs(rgb_image_processed, mask_blue, color='blue')

        # Find centroids of the largest blobs of each mask
        centroid_red, area_red = self.findCentroid(mask_red.astype(np.uint8) * 255)
        centroid_green, area_green = self.findCentroid(mask_green.astype(np.uint8) * 255)
        centroid_blue, area_blue = self.findCentroid(mask_blue.astype(np.uint8) * 255)

        # Annotate the closest players of my_team, my_preys and my_hunters
        if not centroid_red is None:
            rgb_image_processed = cv2.circle(rgb_image_processed, (round(centroid_red[0]), round(centroid_red[1])),
                                             8, (0, 0, 0), -1)
        if not centroid_green is None:
            rgb_image_processed = cv2.circle(rgb_image_processed, (round(centroid_green[0]), round(centroid_green[1])),
                                             8, (0, 0, 0), -1)
            if self.name in self.teams['red_team']:
                self.prey_active = True
        else:
            self.prey_active = False
        if not centroid_blue is None:
            rgb_image_processed = cv2.circle(rgb_image_processed, (round(centroid_blue[0]), round(centroid_blue[1])),
                                             8, (0, 0, 0), -1)

        cv2.imshow('Original image', rgb_image_original)  # Display the image
        cv2.imshow('Processed image', rgb_image_processed)  # Display the image
        cv2.imshow('Red players mask', mask_red.astype(np.uint8) * 255)  # Display the segmented image
        cv2.imshow('Green players mask', mask_green.astype(np.uint8) * 255)  # Display the segmented image
        cv2.imshow('Blue players mask', mask_blue.astype(np.uint8) * 255)  # Display the segmented image
        cv2.waitKey(1)

    def findCentroid(self, mask_original):
        """
        Create a mask with the largest blob of mask_original and return its centroid coordinates
        :param mask_original: Cv2 image - Uint8
        :return mask: Cv2 image - Bool
        :return centroid: List of 2 values
        """

        # Defining maximum area and mask label
        maxArea = 0
        maxLabel = 0

        # You need to choose 4 or 8 for connectivity type
        connectivity = 4

        # Perform the operation
        output = cv2.connectedComponentsWithStats(mask_original, connectivity, cv2.CV_32S)

        # Get the results
        # The first cell is the number of labels
        num_labels = output[0]

        # The second cell is the label matrix
        labels = output[1]

        # The third cell is the stat matrix
        stats = output[2]

        # The fourth cell is the centroid matrix
        centroids = output[3]

        # For each blob, find their area and compare it to the largest one
        for label in range(1, num_labels):
            # Find area
            area = stats[label, cv2.CC_STAT_AREA]

            # If the area is larger then the max area to date, replace it
            if area > maxArea:
                maxArea = area
                maxLabel = label

        # If there are blobs, the label is different than zero
        if maxLabel != 0:
            # Create a new mask and find its centroid
            centroid = centroids[maxLabel]
        else:
            # If there are no blobs, the mask stays the same, and there are no centroids
            centroid = None

        return centroid, maxArea

    def paintBlobs(self, image, mask, color):
        """
        Using a mask, create a green undertone to an image
        :param image: Cv2 image - Uint8
        :param mask: Cv2 image - Bool
        :return image: Cv2 image - Uint8
        """

        # Determine image size
        h, w, _ = image.shape

        if color == 'red':
            # Creating colour channels, using the mask as the green one
            b = np.zeros(shape=[h, w], dtype=np.uint8)
            g = np.zeros(shape=[h, w], dtype=np.uint8)
            r = mask.astype(np.uint8) * 255

            # Merge the channels to create a green mask
            image_specified_color = cv2.merge([b, g, r])

        elif color == 'green':
            # Creating colour channels, using the mask as the green one
            b = np.zeros(shape=[h, w], dtype=np.uint8)
            g = mask.astype(np.uint8) * 255
            r = np.zeros(shape=[h, w], dtype=np.uint8)

            # Merge the channels to create a green mask
            image_specified_color = cv2.merge([b, g, r])

        elif color == 'blue':
            # Creating colour channels, using the mask as the green one
            b = mask.astype(np.uint8) * 255
            g = np.zeros(shape=[h, w], dtype=np.uint8)
            r = np.zeros(shape=[h, w], dtype=np.uint8)

            # Merge the channels to create a green mask
            image_specified_color = cv2.merge([b, g, r])

        # Blend the image with the green mask
        image = cv2.addWeighted(image, 1, image_specified_color, 0.2, 0)

        return image

    def goalReceivedCallback(self, msg):
        """
        callback function that receives a goal pose and transform to odom frame.
        :param msg: goal pose. type: PoseStamped
        """
        print('Received new goal')
        target_frame = self.name + '/odom'  # odom frame is the target frame, always

        # Transform the goal pose to odom frame and active it, if it is not possible, ignore the goal.
        try:
            self.goal = self.tf_buffer.transform(msg, target_frame, rospy.Duration(1))
            self.goal_active = True
            rospy.loginfo('Setting new goal on frame ' + target_frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.goal_active = False
            rospy.logerr('Could not transform goal from ' + msg.header.frame_id + ' to ' + target_frame +
                         '. Ignoring this goal.')

    def sendCommandCallback(self, event):
        """
        Callback function that send the twist command to the robot (speed and angle needed to go to the goal pose)
        :param event: not used.
        """
        # print('Sending twist command')

        # Check if the goal pose is active or not
        if not self.goal_active:  # No goal, no movement
            self.speed = 0
            self.angle = 0.5
        else:
            self.driveStraight()

        # Check if the goal pose is active or not
        if not self.prey_active:  # No prey, rotate along its axis to find a prey
            self.speed = 0
            self.angle = 0.5
        else:
            self.pursuePrey()

        # Construct the twist message for the robot with the speed and angle needed
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angle

        # Publish the twist command to the robot
        self.publisher_command.publish(twist)

    def driveStraight(self, minimum_speed=0.1, maximum_speed=0.75):
        """
        Function that receives the goal pose and calculate the speed and angle to drive to that goal.
        :param minimum_speed: minimum speed allowed to the robot when driving to the goal pose.
        :param maximum_speed: maximum speed allowed to the robot when driving to the goal pose.
        """

        # Receive the goal pose and transform to the base_footprint frame
        goal_copy = copy.deepcopy(self.goal)
        goal_copy.header.stamp = rospy.Time.now()
        goal_in_base_link = self.tf_buffer.transform(goal_copy, self.name + '/base_footprint', rospy.Duration(1))

        # Get x and y coordinates of the goal pose
        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y

        # Compute the angle needed to the robot goes to the goal pose.
        self.angle = math.atan2(y, x)

        # Calculate distance to goal and vary the speed according to it. Speed to go to the goal pose.
        distance_to_goal = math.sqrt(x ** 2 + y ** 2)
        self.speed = max(minimum_speed, 0.5 * distance_to_goal)  # limit minimum speed
        self.speed = min(maximum_speed, self.speed)  # limit maximum speed

        # if the robot is close enough to the goal pose, stop the robot. This is to avoid the robot drive in circles
        # continuously
        if distance_to_goal < 0.08:
            self.goal_active = False
            rospy.loginfo('Robot achieved its goal.')

    def pursuePrey(self, minimum_speed=0.1, maximum_speed=0.75):
        """
        Function that receives the goal pose and calculate the speed and angle to drive to that goal.
        :param minimum_speed: minimum speed allowed to the robot when driving to the goal pose.
        :param maximum_speed: maximum speed allowed to the robot when driving to the goal pose.
        """

        my_prey_color = self.teams['my_preys'][0][:-1]
        print(my_prey_color)

        centroid_copy = copy.deepcopy(self.goal)


def publisher():
    # ----------------------------------
    # Initialization
    # ----------------------------------

    # Initialize node and call class instance
    rospy.init_node('p_ldalcol_driver', anonymous=False)  # Initialize the node
    driver = Driver()

    rate = rospy.Rate(10)  # time rate of the message

    rospy.spin()

    # # ----------------------------------
    # # Execution
    # # ----------------------------------
    # while not rospy.is_shutdown():
    #     twist = Twist()
    #     twist.linear.x = 0.1
    #     twist.angular.z = -1
    #
    #     publisher.publish(twist)
    #     rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
