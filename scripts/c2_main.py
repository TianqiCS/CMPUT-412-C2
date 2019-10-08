#!/usr/bin/env python
from math import copysign

import rospy, cv2, cv_bridge, numpy
from tf.transformations import decompose_matrix, euler_from_quaternion
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_numpy import numpify
import numpy as np

from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound

import smach
import smach_ros

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
from sensor_msgs.msg import Joy

from nav_msgs.srv import SetMap
from nav_msgs.msg import OccupancyGrid

current_twist = Twist()
total_redline = 0
red = False
rospy.init_node('c2_main')


twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
led_pub_1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
led_pub_2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

current_time = rospy.Time.now()

class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'end'])
    def execute(self, userdata):
        global total_redline, red
        if not red:
            twist_pub.publish(current_twist)
            return 'running'
        else:
            twist = Twist()
            twist_pub.publish(twist)
            rospy.sleep(0.5)
            return 'end'

class PassThrough(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'end'])
    def execute(self, userdata):
        global total_redline, red
        if red:
            twist_pub.publish(current_twist)
            return 'running'
        else:
            return 'end'        

class SmCore:
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=['end'])
        self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')

        self.sis.start()
        with self.sm:
            smach.StateMachine.add('Follow', Follow(),
                                    transitions={'running':'Follow',
                                                'end':'PassThrough'})
            smach.StateMachine.add('PassThrough', PassThrough(),
                                    transitions={'running':'PassThrough',
                                                'end':'Follow'})                    

            self.bridge = cv_bridge.CvBridge()

            self.integral = 0
            self.previous_error = 0

            self.Kp = - 1 / 200.0
            self.Kd = 1 / 3000.0
            self.Ki = 0.0

            rospy.Subscriber('usb_cam/image_raw', Image, self.usb_image_callback)
            #rospy.Subscriber('camera/rgb/image_raw', Image, self.kinect_image_callback)
            #rospy.Subscriber("/joy", Joy, self.joy_callback)

    def usb_image_callback(self, msg):
        global stop_line_flag, flag_line_flag, counter_loc1, counter_loc2, object_type, backing_flag, current_type, max_linear_vel, time_after_stop, is_end_of_line, is_moving_loc2, is_end_loc2, total_redline, red
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #image = cv2.flip(image, -1)  ### flip
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # white color
        lower_white = numpy.array([0, 0, 170])
        upper_white = numpy.array([360, 30, 255])

        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = image.shape
        search_top = 3 * h / 4 + 20
        search_bot = 3 * h / 4 + 30
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        M = cv2.moments(mask)

        if M['m00'] > 0:
            self.cx_white = int(M['m10'] / M['m00'])
            self.cy_white = int(M['m01'] / M['m00'])
            cv2.circle(image, (self.cx_white, self.cy_white), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = self.cx_white - w / 2
            current_twist.linear.x = 0.5  # and <= 1.7

            self.integral = self.integral + err * 0.05
            self.derivative = (err - self.previous_error) / 0.05

            current_twist.angular.z = float(err) * self.Kp + (self.Ki * float(self.integral)) + (
                    self.Kd * float(self.derivative))

            self.previous_error = err

        
        # usb red

        lower_red = numpy.array([0, 100, 100])
        upper_red = numpy.array([360, 256, 256])

        # if loc3_stop_time == 0:
        #     lower_red = numpy.array([0, 150, 50])
        #     upper_red = numpy.array([360, 256, 256])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        h, w, d = image.shape

        search_top = h - 40
        search_bot = h - 1

        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:

            for item in contours:
                area = cv2.contourArea(item)

                if area > 5000:
                    M = cv2.moments(item)
                    self.cx_red = int(M['m10'] / M['m00'])
                    self.cy_red = int(M['m01'] / M['m00'])
                    (x, y), radius = cv2.minEnclosingCircle(item)
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(image, center, radius, (0, 255, 0), 2)

                    total_redline += 1
                    red = True

                    #rospy.sleep(2)


                elif area > 1000:
                    M = cv2.moments(item)
                    self.cx_red = int(M['m10'] / M['m00'])
                    self.cy_red = int(M['m01'] / M['m00'])
                    (x, y), radius = cv2.minEnclosingCircle(item)
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(image, center, radius, (0, 255, 0), 2)

                    total_redline += 1
                    red = True

                    #rospy.sleep(2)

                else:
                    red = False

        # red_mask = cv2.inRange(hsv, lower_red, upper_red)

        cv2.imshow("refer_dot", image)
        cv2.waitKey(3)

    def execute(self):
        outcome = self.sm.execute()
        rospy.spin()
        self.sis.stop()

    
c = SmCore()
c.execute()