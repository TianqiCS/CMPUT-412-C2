#!/usr/bin/env python
import math, time
from math import copysign

import rospy, cv2, cv_bridge, numpy
from tf.transformations import decompose_matrix, euler_from_quaternion
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from ros_numpy import numpify
import numpy as np

from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound

import smach
import smach_ros

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
#from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
from sensor_msgs.msg import Joy

from nav_msgs.srv import SetMap
from nav_msgs.msg import OccupancyGrid

from detectshapes import ContourDetector
from detectshapes import Contour


def approxEqual(a, b, tol = 0.001):
    return abs(a - b) <= tol

def publish_sound_led(quantity):
    global sound_pub, led_pub_1, led_pub_2
    if quantity == 1:
        led_pub_1.publish(Led.GREEN)
    elif quantity == 2:
        led_pub_2.publish(Led.GREEN)
    else:
        led_pub_1.publish(Led.GREEN)
        led_pub_2.publish(Led.GREEN)

    for i in range(quantity):
        sound_pub.publish(1)
        rospy.sleep(1)
    
    led_pub_1.publish(Led.BLACK)
    led_pub_2.publish(Led.BLACK)

class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'end', 'turning'])

    def execute(self, userdata):
        global stop, turn, twist_pub, current_work

        if turn:
            return 'turning'
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
        global stop, twist_pub
        if stop:
            twist_pub.publish(current_twist)
            return 'running'
        else:
            return 'end'

class Rotate(smach.State):
    def __init__(self):
        self.unit = math.pi/2  # 90 degrees
        smach.State.__init__(self, 
                                outcomes=['running','working1','working2','working3','end'],
                                input_keys=['rotate_turns_in'],
        )

    def execute(self, userdata):
        global g_odom, turn, work, current_work, twist_pub, on_additional_line
        init_yaw = g_odom['yaw_z']
        target_yaw = init_yaw + userdata.rotate_turns_in * self.unit
        if target_yaw >  math.pi:
            target_yaw = target_yaw - 2 * math.pi
        if target_yaw < -1 * math.pi:
            target_yaw = target_yaw + 2 * math.pi
        twist = Twist()
        #on_additional_line = True #debug
        while True:
            if approxEqual(g_odom['yaw_z'], target_yaw):
                turn = False
                if work:
                    if current_work == 1:
                        return 'working1'
                    elif current_work == 2:
                        on_additional_line = True
                        return 'working2'
                    else:
                        return 'working3'
                else:
                    if on_additional_line:
                        return 'working2'
                    else:
                        work = True
                        return 'end'

            elif g_odom['yaw_z']> target_yaw:
                twist.angular.z = -0.3
            else:
                twist.angular.z = 0.3
            #print g_odom['yaw_z'], target_yaw
            twist_pub.publish(twist)
        return 'running'

class TaskControl(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['end'],
                                output_keys=['rotate_turns'])

    def execute(self, userdata):
        global current_work
        userdata.rotate_turns = 1
        return 'end'

class Work1(smach.State):
    def __init__(self):
        self.hsv = None
        smach.State.__init__(self, 
                                outcomes=['rotate'],
                                output_keys=['rotate_turns']
        )

    def execute(self, userdata):
        global work, current_work
        self.observe()
        work = False
        current_work += 1
        userdata.rotate_turns = -1
        time.sleep(0.5)
        return 'rotate'
    
    def observe(self):
        global twist_pub

        cd = ContourDetector()
        image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.shape_cam_callback)
        task_done = False
        start_time = time.time()
        while not task_done and time.time() - start_time < 5:
            if self.hsv != None:
                contours1 = cd.getContours(self.hsv, "red", 1)
                twist = Twist()
                twist.angular.z = 0.1
                twist_pub.publish(twist)
                rospy.sleep(0.5)
                contours2 = cd.getContours(self.hsv, "red", 1)
                twist.angular.z = -0.1
                twist_pub.publish(twist)
                if len(contours1) == len(contours2):
                    print "numer of objects:", len(contours1)
                    publish_sound_led(len(contours1))
                    task_done = True
        if task_done == False:
            contours = cd.getContours(self.hsv, "red", 1)
            number = 3 if len(contours) == 0 else len(contours)
            print "Failed Handle: number of objects: ", number 
            publish_sound_led(number)
        image_sub.unregister()

    def shape_cam_callback(self, msg):
        bridge = cv_bridge.CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

class Work2(smach.State):
    def __init__(self):
        self.hsv = None
        smach.State.__init__(self, 
                                outcomes=['rotate'],
                                output_keys=['rotate_turns']
        )

    def execute(self, userdata):
        global work, current_work
        self.observe()
        work = False
        current_work += 1
        userdata.rotate_turns = -2
        return 'rotate'
    
    def observe(self):
        global shape_at_loc2, twist_pub

        cd = ContourDetector()
        image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.shape_cam_callback)

        task_done = False
        start_time = time.time()
        while not task_done and time.time() - start_time < 5:
            if self.hsv != None:
                contours1 = cd.getContours(self.hsv, "red and green", 2)
                twist = Twist()
                twist.angular.z = 0.1
                twist_pub.publish(twist)
                rospy.sleep(0.5)
                contours2 = cd.getContours(self.hsv, "red and green", 2)
                twist.angular.z = -0.1
                twist_pub.publish(twist)
                if len(contours1) == len(contours2):
                    print "number of objects:", len(contours1)
                    publish_sound_led(len(contours1))
                    task_done = True
        if task_done == False:
            contours = cd.getContours(self.hsv, "red and green", 1)
            number = 3 if len(contours) == 0 else len(contours)
            print "Failed Handle: number of objects: ", number
            publish_sound_led(number)
        
        task_done = False
        start_time = time.time()
        while not task_done and time.time() - start_time < 5:
            if self.hsv != None:
                contours1 = cd.getContours(self.hsv, "green", 2)
                twist = Twist()
                twist.angular.z = 0.1
                twist_pub.publish(twist)
                rospy.sleep(0.5)
                contours2 = cd.getContours(self.hsv, "green", 2)
                twist.angular.z = -0.1
                twist_pub.publish(twist)
                if len(contours1) == len(contours2) and len(contours1) > 0:
                    if contours1[0] == contours2[0]:
                        shape_at_loc2 = contours1[0]
                        print "shape at loc2: ", shape_at_loc2
                        task_done = True
        if task_done == False:
            contours = cd.getContours(self.hsv, "green", 2)
            shape_at_loc2 = Contour.Circle if len(contours) == 0 else contours[0]
            print "Failed Handle: shape at loc2: ", shape_at_loc2

        image_sub.unregister()

    def shape_cam_callback(self, msg):
        bridge = cv_bridge.CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

class Work2Follow(smach.State):
    def __init__(self):
        self.w2_twist = Twist()
        self.w2_stop = False
        self.w2_integral = 0
        self.w2_previous_error = 0

        self.w2_Kp = - 2 / 200.0
        self.w2_Kd = 1 / 3000.0
        self.w2_Ki = 0.0
        smach.State.__init__(self, 
                                outcomes=['arrived', 'returned', 'running'],
                                output_keys=['rotate_turns']
        )
        
    def execute(self, userdata):
        global twist_pub, current_work, on_additional_line
        time.sleep(2)
        self.w2_stop = False
        #if current_work < 2: #debug
        #    current_work = 2 #debug 
        while not self.w2_stop:
            self.control_speed()
        if current_work == 2:
            return 'arrived'
        else:
            userdata.rotate_turns = 1
            on_additional_line = False
            return 'returned'
    
    def control_speed(self):
        global white_mask, red_mask, g_odom, twist_pub, image_width, current_work

        M_white = cv2.moments(white_mask)
        M_red = cv2.moments(red_mask)

        if M_white['m00'] > 0:
            cx= int(M_white['m10'] / M_white['m00'])

            # BEGIN CONTROL
            err = cx - image_width / 2
            self.w2_twist.linear.x = 0.5  # and <= 1.7

            self.w2_integral = self.w2_integral + err * 0.05
            self.w2_derivative = (err - self.w2_previous_error) / 0.05

            self.w2_twist.angular.z = float(err) * self.w2_Kp + (self.w2_Ki * float(self.w2_integral)) + (
                    self.w2_Kd * float(self.w2_derivative))

            self.w2_previous_error = err

            twist_pub.publish(self.w2_twist)
            print 'saw white', self.w2_twist.angular.z
        else:
            print 'no white'
            if current_work == 3:
                time.sleep(1)
            self.w2_stop = True
            if M_red['m00'] > 0 and current_work == 3:
                print 'saw red'
                ori_x = g_odom['x']
                ori_y = g_odom['y']
                twist = Twist()
                twist.linear.x = 0.1
                while True:
                    twist_pub.publish(twist)
                    if abs(ori_x - g_odom['x']) > 0.15 or abs(ori_y - g_odom['y']) > 0.15:
                        break

class Work3(smach.State):
    def __init__(self):
        self.hsv = None
        smach.State.__init__(self, 
                                outcomes=['rotate'],
                                output_keys=['rotate_turns']
        )
        
    def execute(self, userdata):
        global work, current_work
        self.observe()
        work = False
        userdata.rotate_turns = -1
        return 'rotate'
    
    def observe(self):
        global current_work, twist_pub, shape_at_loc2, redline_count_loc3
        redline_count_loc3 += 1
        cd = ContourDetector()
        image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.shape_cam_callback)

        task_done = False
        start_time = time.time()
        while not task_done and time.time() - start_time < 5:
            if self.hsv != None:
                contours1 = cd.getContours(self.hsv, "red", 2)
                twist = Twist()
                twist.angular.z = 0.1
                twist_pub.publish(twist)
                rospy.sleep(0.5)
                contours2 = cd.getContours(self.hsv, "red", 2)
                twist.angular.z = -0.1
                twist_pub.publish(twist)
                if len(contours1) == len(contours2) and len(contours1) > 0:
                    if contours1[0] == contours2[0] and contours1[0] == shape_at_loc2:
                        print "Found ", contours1[0]
                        publish_sound_led(1)
                    else:
                        print "Not Matching"
                    task_done = True

        if task_done == False and redline_count_loc3 == 3:
            print "Failed Handle: Found", shape_at_loc2
            publish_sound_led(1)

        image_sub.unregister()

    def shape_cam_callback(self, msg):
        bridge = cv_bridge.CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

class SmCore:
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=['end'])
        #self.sm.userdata.turn = 0
        self.sm.userdata.turns = 0
        self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')

        self.sis.start()
        with self.sm:   
            smach.StateMachine.add('Follow', Follow(),
                                    transitions={'running':'Follow',
                                                'end':'PassThrough',
                                                'turning':'TaskControl'
                                                })
            smach.StateMachine.add('PassThrough', PassThrough(),
                                    transitions={'running':'PassThrough',
                                                'end':'Follow'})                    
            smach.StateMachine.add('Rotate', Rotate(),
                                    transitions={'running':'Rotate',
                                                'working1': 'Work1',
                                                'working2': 'Work2Follow',
                                                'working3': 'Work3',
                                                'end': 'Follow'},
                                    remapping={'rotate_turns_in':'turns'})
            smach.StateMachine.add('TaskControl', TaskControl(),
                                    transitions={'end':'Rotate'},
                                    remapping={'rotate_turns':'turns'})
            smach.StateMachine.add('Work1', Work1(),
                                    transitions={'rotate':'Rotate'},
                                    remapping={'rotate_turns':'turns'})
            smach.StateMachine.add('Work2', Work2(),
                                    transitions={'rotate':'Rotate'},
                                    remapping={'rotate_turns':'turns'}) 
            smach.StateMachine.add('Work2Follow', Work2Follow(),
                                    transitions={'running':'Work2Follow',
                                                'arrived':'Work2',
                                                'returned':'Rotate'},
                                    remapping={'rotate_turns':'turns'})
            smach.StateMachine.add('Work3', Work3(),
                                    transitions={'rotate':'Rotate'},
                                    remapping={'rotate_turns':'turns'})                                                              
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
        global stop, turn, current_work, white_mask, red_mask, image_width

        full_red_line = False

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #image = cv2.flip(image, -1)  ### flip
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # white color mask
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([360, 30, 255])

        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        h, w, _ = image.shape
        image_width = w
        search_top = 3 * h / 4 + 20
        search_bot = 3 * h / 4 + 30
        white_mask[0:search_top, 0:w] = 0
        white_mask[search_bot:h, 0:w] = 0

        # red color mask
        lower_red = numpy.array([0, 100, 100])
        upper_red = numpy.array([360, 256, 256])

        red_mask = cv2.inRange(hsv, lower_red, upper_red)

        h, w, d = image.shape

        search_top = h - 40
        search_bot = h - 1

        red_mask[0:search_top, 0:w] = 0
        red_mask[search_bot:h, 0:w] = 0


        M = cv2.moments(white_mask)

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
        
        im2, contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
                    if self.cx_white == 0: #full red line
                        full_red_line = True
                        stop = True
                    elif x + radius < self.cx_white: #half red line
                        if "Rotate" not in self.sm.get_active_states():
                            rospy.sleep(0.5)
                            turn = True
                elif "PassThrough" in self.sm.get_active_states():
                    stop = False

            cv2.imshow("refer_dot", image)
            cv2.waitKey(3)
            #print stop, turn
    def execute(self):
        outcome = self.sm.execute()
        rospy.spin()
        self.sis.stop()

current_twist = Twist()
stop = False
turn = False
work = True
shape_at_loc2 = None
redline_count_loc3 = 0
current_work = 1
on_additional_line = False
white_mask = None
red_mask = None
image_width = 0
g_odom = {'x':0.0, 'y':0.0, 'yaw_z':0.0}

rospy.init_node('c2_main')

twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
led_pub_1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
led_pub_2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)
    
c = SmCore()
c.execute()