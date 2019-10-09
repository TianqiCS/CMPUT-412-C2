#!/usr/bin/env python
import math
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
stop = False
work = False
turn = False
current_work = 0
g_odom = {'x':0.0, 'y':0.0, 'yaw_z':0.0}
rospy.init_node('c2_main')


twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
led_pub_1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
led_pub_2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

current_time = rospy.Time.now()

def approxEqual(a, b, tol = 0.001):
    return abs(a - b) <= tol

class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'end', 'task1'])
    def execute(self, userdata):
        global total_redline, stop, turn
        if turn:
            return 'task1'
        if not stop:
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
        global total_redline, stop
        if stop:
            twist_pub.publish(current_twist)
            return 'running'
        else:
            return 'end'

class Rotate(smach.State):
    def __init__(self):
        self.unit = math.pi/2  # 90 degrees
        smach.State.__init__(self, 
                                outcomes=['running','end'],
                                input_keys=['rotate_turns_in'],
                                output_keys=['rotate_turns_out']
        )
    def execute(self, userdata):
        global g_odom, turn
        init_yaw = g_odom['yaw_z']
        target_yaw = init_yaw + userdata.rotate_turns_in * self.unit
        if target_yaw >  math.pi:
            target_yaw = target_yaw - 2 * math.pi
        if target_yaw < -1 * math.pi:
            target_yaw = target_yaw + 2 * math.pi
        twist = Twist()
        while True:
            if approxEqual(g_odom['yaw_z'], target_yaw):
                return 'end'

            elif g_odom['yaw_z'] > target_yaw:
                twist.angular.z = -0.3
            else:
                twist.angular.z = 0.3
            twist_pub.publish(twist)
        return 'running'

class Task1(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['turning', 'working', 'end'],
                                input_keys=['rotate_turns_in', 'task_worked_in'],
                                output_keys=['rotate_turns_out']
        )
    def execute(self, userdata):
        global turn, work, current_work
        if not work and turn:
            turn = False
            userdata.rotate_turns_out = 1
            return 'turning'
        elif not work and not turn:
            turn = True
            return 'working'
        elif work and turn:
            turn = False
            userdata.rotate_turns_out = -1
            return 'turning'
        elif work and not turn:
            turn = False
            work = False
            current_work += 1
            return 'end'

class Work(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['end'],
                                input_keys=['task_worked_in'],
                                output_keys=['task_worked_out']
        )
    def execute(self, userdata):
        global work
        rospy.sleep(3)
        work = True
        userdata.task_worked_out = 1
        return 'end'

class SmCore:
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=['end'])
        self.sm.userdata.task1 = False
        self.sm.userdata.turn = 0
        self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')

        self.sis.start()
        with self.sm:
            smach.StateMachine.add('Follow', Follow(),
                                    transitions={'running':'Follow',
                                                'end':'PassThrough',
                                                'task1':'Task1'
                                                })
            smach.StateMachine.add('PassThrough', PassThrough(),
                                    transitions={'running':'PassThrough',
                                                'end':'Follow'})                    
            smach.StateMachine.add('Rotate', Rotate(),
                                    transitions={'running':'Rotate',
                                                'end':'Task1'},
                                    remapping={'rotate_turns_in':'turns', 
                                               'rotate_turns_out':'turns'})
            smach.StateMachine.add('Task1', Task1(),
                                    transitions={'working':'Work',
                                                'turning':'Rotate',
                                                'end':'Follow'},
                                    remapping={'rotate_turns_in':'turns', 
                                               'rotate_turns_out':'turns', 
                                               'task_worked_in':'task1'})
            smach.StateMachine.add('Work', Work(),
                                    transitions={'end':'Task1'},
                                    remapping={'task_worked_in':'task1', 
                                               'task_worked_out':'task1'})                                                           
            self.bridge = cv_bridge.CvBridge()

            self.integral = 0
            self.previous_error = 0

            self.Kp = - 1 / 200.0
            self.Kd = 1 / 3000.0
            self.Ki = 0.0

            rospy.Subscriber('usb_cam/image_raw', Image, self.usb_image_callback)
            rospy.Subscriber('odom', Odometry, self.odom_callback)
            #rospy.Subscriber('camera/rgb/image_raw', Image, self.kinect_image_callback)
            #rospy.Subscriber("/joy", Joy, self.joy_callback)

    def odom_callback(self, msg):
        global g_odom
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])
        
        g_odom['x'] = x
        g_odom['y'] = y
        g_odom['yaw_z'] = yaw[2]

    def usb_image_callback(self, msg):
        global stop_line_flag, flag_line_flag, counter_loc1, counter_loc2, object_type, backing_flag, current_type, max_linear_vel, time_after_stop, is_end_of_line, is_moving_loc2, is_end_loc2, total_redline, stop, turn
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #image = cv2.flip(image, -1)  ### flip
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # white color
        lower_white = numpy.array([0, 0, 200])
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
        else:
            self.cx_white = 0


        

        
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
                    if self.cx_white == 0:
                        stop = True
                    elif x + radius < self.cx_white:
                        if "Rotate" not in self.sm.get_active_states():
                            rospy.sleep(0.5)
                            turn = True
                elif "PassThrough" in self.sm.get_active_states():
                    stop = False

        # red_mask = cv2.inRange(hsv, lower_red, upper_red)

        cv2.imshow("refer_dot", image)
        cv2.waitKey(3)

    def execute(self):
        outcome = self.sm.execute()
        rospy.spin()
        self.sis.stop()

    
c = SmCore()
c.execute()