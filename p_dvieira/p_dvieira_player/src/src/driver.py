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
from colorama import Fore
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from psr_parte09_exs.msg import Dog
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PoseStamped
from sensor_msgs.msg import Image


class Driver:
    def __init__(self):
        # Define all variables
        self.biggest_area_green = None
        self.biggest_centroid_blue = None
        self.biggest_area_blue = None
        self.biggest_area_red = None
        self.biggest_centroid_red = None
        self.teams = None
        self.biggest_centroid_green = None
        self.name = rospy.get_name()
        self.name = self.name.strip('/')  # remove initial /
        self.goal = PoseStamped()
        self.goal_active = False
        self.angle = 0
        self.speed = 0
        self.image_center = 427
        self.debug = rospy.get_param('/debug')
        self.ranges_red = {'b': {'min': 0, 'max': 50},
                           'g': {'min': 0, 'max': 50},
                           'r': {'min': 100, 'max': 256}}
        self.ranges_green = {'b': {'min': 0, 'max': 50},
                             'g': {'min': 100, 'max': 256},
                             'r': {'min': 0, 'max': 50}}
        self.ranges_blue = {'b': {'min': 100, 'max': 256},
                            'g': {'min': 0, 'max': 50},
                            'r': {'min': 0, 'max': 50}}
        self.mins_red = np.array(
            [self.ranges_red['b']['min'], self.ranges_red['g']['min'], self.ranges_red['r']['min']])
        self.maxs_red = np.array(
            [self.ranges_red['b']['max'], self.ranges_red['g']['max'], self.ranges_red['r']['max']])
        self.mins_green = np.array(
            [self.ranges_green['b']['min'], self.ranges_green['g']['min'], self.ranges_green['r']['min']])
        self.maxs_green = np.array(
            [self.ranges_green['b']['max'], self.ranges_green['g']['max'], self.ranges_green['r']['max']])
        self.mins_blue = np.array(
            [self.ranges_blue['b']['min'], self.ranges_blue['g']['min'], self.ranges_blue['r']['min']])
        self.maxs_blue = np.array(
            [self.ranges_blue['b']['max'], self.ranges_blue['g']['max'], self.ranges_blue['r']['max']])
        # You need to choose 4 or 8 for connectivity type
        self.connectivity = 4
        self.state = 'waiting'
        self.distance_to_center = 0

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
        self.my_prey_color = self.teams['my_preys'].split('_')[0]
        self.my_hunter_color = self.teams['my_hunters'].split('_')[0]

        # Initialize the detection of the robots through computer vision
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber('/' + self.name + '/camera/rgb/image_raw', Image,
                                                 self.detectRobotsWithVisionCallback)

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

    def detectRobotsWithVisionCallback(self, image_msg):
        # Convert the from imgmsg ROS to opencv.
        try:
            rgb_image_original = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # Segment the color for red players
        # Processing
        mask_red = cv2.inRange(rgb_image_original, self.mins_red, self.maxs_red)
        # conversion from numpy from uint8 to bool
        mask_red = mask_red.astype(bool)

        # Segment the color for green players
        # Processing
        mask_green = cv2.inRange(rgb_image_original, self.mins_green, self.maxs_green)
        # conversion from numpy from uint8 to bool
        mask_green = mask_green.astype(bool)

        # Segment the color for blue players
        # Processing
        mask_blue = cv2.inRange(rgb_image_original, self.mins_blue, self.maxs_blue)
        # conversion from numpy from uint8 to bool
        mask_blue = mask_blue.astype(bool)

        # Find centroids of the largest blobs of each mask
        self.biggest_centroid_red, self.biggest_area_red = self.findCentroid(mask_red.astype(np.uint8) * 255)
        self.biggest_centroid_green, self.biggest_area_green = self.findCentroid(mask_green.astype(np.uint8) * 255)
        self.biggest_centroid_blue, self.biggest_area_blue = self.findCentroid(mask_blue.astype(np.uint8) * 255)

        # Decide state mode
        if self.name in self.teams['red_team']:
            # If hunter and prey are detected at the same time, it should be decided which mode will be activated.
            if self.biggest_centroid_green is not None and self.biggest_centroid_blue is not None:
                # If green blob is bigger, activate hunting mode. If not, activate escaping mode. The bigger the blob,
                # the closest the prey or the hunter are.
                if self.biggest_area_green > self.biggest_area_blue:
                    self.state = 'hunting'
                else:
                    self.state = 'escaping'

            # Check if there is only hunter or prey detected.
            elif self.biggest_centroid_green is not None or self.biggest_centroid_blue is not None:
                # Activate hunting mode
                if self.biggest_centroid_green is not None:
                    self.state = 'hunting'
                # Activate escape mode
                elif self.biggest_centroid_blue is not None:
                    self.state = 'escaping'

            # If there is no hunter or prey detected, active waiting mode.
            else:
                self.state = 'waiting'

        if self.name in self.teams['green_team']:
            # If hunter and prey are detected at the same time, it should be decided which mode will be activated.
            if self.biggest_centroid_blue is not None and self.biggest_centroid_red is not None:
                # If green blob is bigger, activate hunting mode. If not, activate escaping mode. The bigger the blob,
                # the closest the prey or the hunter are.
                if self.biggest_area_blue > self.biggest_area_red:
                    self.state = 'hunting'
                else:
                    self.state = 'escaping'

            # Check if there is only hunter or prey detected.
            elif self.biggest_centroid_blue is not None or self.biggest_centroid_red is not None:
                # Activate hunting mode
                if self.biggest_centroid_blue is not None:
                    self.state = 'hunting'
                # Activate escape mode
                elif self.biggest_centroid_red is not None:
                    self.state = 'escaping'

            # If there is no hunter or prey detected, active waiting mode.
            else:
                self.state = 'waiting'

        if self.name in self.teams['blue_team']:
            # If hunter and prey are detected at the same time, it should be decided which mode will be activated.
            if self.biggest_centroid_red is not None and self.biggest_centroid_green is not None:
                # If green blob is bigger, activate hunting mode. If not, activate escaping mode. The bigger the blob,
                # the closest the prey or the hunter are.
                if self.biggest_area_red > self.biggest_area_green:
                    self.state = 'hunting'
                else:
                    self.state = 'escaping'

            # Check if there is only hunter or prey detected.
            elif self.biggest_centroid_red is not None or self.biggest_centroid_green is not None:
                # Activate hunting mode
                if self.biggest_centroid_red is not None:
                    self.state = 'hunting'
                # Activate escape mode
                elif self.biggest_centroid_green is not None:
                    self.state = 'escaping'

            # If there is no hunter or prey detected, active waiting mode.
            else:
                self.state = 'waiting'

        # Annotate the closest players of my_team, my_preys and my_hunters
        if self.debug is True:
            # Paint largest blobs
            rgb_image_processed = copy.deepcopy(rgb_image_original)
            rgb_image_processed = self.paintBlobs(rgb_image_processed, mask_red, color='red')
            rgb_image_processed = self.paintBlobs(rgb_image_processed, mask_green, color='green')
            rgb_image_processed = self.paintBlobs(rgb_image_processed, mask_blue, color='blue')

            if self.biggest_centroid_red is not None:
                rgb_image_processed = cv2.circle(rgb_image_processed,
                                                 (round(self.biggest_centroid_red[0]),
                                                  round(self.biggest_centroid_red[1])),
                                                 8, (0, 0, 0), -1)

            if self.biggest_centroid_green is not None:
                rgb_image_processed = cv2.circle(rgb_image_processed,
                                                 (round(self.biggest_centroid_green[0]),
                                                  round(self.biggest_centroid_green[1])),
                                                 8, (0, 0, 0), -1)

            if self.biggest_centroid_blue is not None:
                rgb_image_processed = cv2.circle(rgb_image_processed,
                                                 (round(self.biggest_centroid_blue[0]),
                                                  round(self.biggest_centroid_blue[1])),
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

        # Perform the operation
        output = cv2.connectedComponentsWithStats(mask_original, self.connectivity, cv2.CV_32S)

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
            self.angle = 0
        else:
            self.driveStraight()

        if self.name in self.teams['red_team'] or self.name in self.teams['green_team'] or self.name in self.teams['blue_team']:
            # If there is a prey detected, pursue prey
            if self.state == 'waiting':
                if self.distance_to_center >= 0:
                    self.speed = 0
                    self.angle = -0.5  # rotate to right
                else:
                    self.speed = 0
                    self.angle = 0.5  # rotate to left

                # if self.debug:
                print(Fore.BLUE + 'My name is ' + self.name + ' and I am waiting for my next prey!!!' + Fore.RESET)

            elif self.state == 'hunting':
                self.pursuePrey(self.my_prey_color)
                # if self.debug:
                print(Fore.GREEN + 'My name is ' + self.name + ' and I am hunting a ' + self.my_prey_color + ' player now!!!' + Fore.RESET)

            elif self.state == 'escaping':
                self.escapeHunter(self.my_hunter_color)
                # if self.debug:
                print(Fore.RED + 'My name is ' + self.name + ' and I have to escape from a ' + self.my_hunter_color + ' player now!!!' + Fore.RESET)

        # Construct the twist message for the robot with the speed and angle needed
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angle

        # Publish the twist command to the robot
        self.publisher_command.publish(twist)

        # # If there is a hunter, escape from him for 2 seconds to maintain the speed and angle commands.
        # if self.state == 'escaping':
        #     rospy.sleep(5)

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

    def pursuePrey(self, my_prey_color, minimum_speed=0.3, maximum_speed=0.9):
        """
        Function that receives the goal pose and calculate the speed and angle to drive to that goal.
        :param my_prey_color:
        :param minimum_speed: minimum speed allowed to the robot when driving to the goal pose.
        :param maximum_speed: maximum speed allowed to the robot when driving to the goal pose.
        """

        if my_prey_color == 'red':
            if self.biggest_centroid_red is not None:
                self.speed = max(minimum_speed, self.biggest_area_red/10000)  # limit minimum speed
                self.speed = min(maximum_speed, self.speed)  # limit maximum speed

                self.distance_to_center = round(self.biggest_centroid_red[0]) - self.image_center
                if self.distance_to_center > 0:  # turn right
                    self.angle = -0.4
                elif self.distance_to_center < 0:  # turn left
                    self.angle = 0.4
                else:  # go straight away to the prey
                    self.angle = 0

        elif my_prey_color == 'green':
            if self.biggest_centroid_green is not None:
                self.speed = max(minimum_speed, self.biggest_area_green/10000)  # limit minimum speed
                self.speed = min(maximum_speed, self.speed)  # limit maximum speed

                self.distance_to_center = round(self.biggest_centroid_green[0]) - self.image_center
                if self.distance_to_center > 0:  # turn right
                    self.angle = -0.4
                elif self.distance_to_center < 0:  # turn left
                    self.angle = 0.4
                else:  # go straight away to the prey
                    self.angle = 0

        elif my_prey_color == 'blue':
            if self.biggest_centroid_blue is not None:
                self.speed = max(minimum_speed, self.biggest_area_blue/10000)  # limit minimum speed
                self.speed = min(maximum_speed, self.speed)  # limit maximum speed

                self.distance_to_center = round(self.biggest_centroid_blue[0]) - self.image_center
                if self.distance_to_center > 0:  # turn right
                    self.angle = -0.4
                elif self.distance_to_center < 0:  # turn left
                    self.angle = 0.4
                else:  # go straight away to the prey
                    self.angle = 0

    def escapeHunter(self, my_hunter_color, minimum_speed=0.3, maximum_speed=0.6):
        """
        Function that receives the goal pose and calculate the speed and angle to drive to that goal.
        :param minimum_speed:
        :param my_hunter_color:
        :param maximum_speed: maximum speed allowed to the robot when driving to the goal pose.
        """

        if my_hunter_color == 'red':
            if self.biggest_centroid_red is not None:
                self.distance_to_center = round(self.biggest_centroid_red[0]) - self.image_center
                if self.distance_to_center > 0:  # turn left, to the opposite direction
                    self.angle = 0.6
                elif self.distance_to_center < 0:  # turn right, to the opposite direction
                    self.angle = -0.6

        elif my_hunter_color == 'green':
            if self.biggest_centroid_green is not None:
                self.distance_to_center = round(self.biggest_centroid_green[0]) - self.image_center
                if self.distance_to_center > 0:  # turn left, to the opposite direction
                    self.angle = 0.6
                elif self.distance_to_center < 0:  # turn right, to the opposite direction
                    self.angle = -0.6

        elif my_hunter_color == 'blue':
            if self.biggest_centroid_blue is not None:
                self.distance_to_center = round(self.biggest_centroid_blue[0]) - self.image_center
                if self.distance_to_center > 0:  # turn left, to the opposite direction
                    self.angle = 0.6
                elif self.distance_to_center < 0:  # turn right, to the opposite direction
                    self.angle = -0.6

        self.speed = maximum_speed


def publisher():
    # ----------------------------------
    # Initialization
    # ----------------------------------

    # Initialize node and call class instance
    rospy.init_node('p_ldalcol_driver', anonymous=False)  # Initialize the node
    driver = Driver()

    rate = rospy.Rate(50)  # time rate of the message

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
