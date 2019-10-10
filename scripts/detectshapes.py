#Referenced from : https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/

import imutils, numpy
import cv2, cv_bridge
from enum import Enum

class Contour(Enum):
    Unidentified = 0
    Triangle = 1
    Rectangle = 2
    Circle = 3

class ContourDetector():
    def __init__(self):
        pass

    def getContours(self, hsv, color = "red", loc = 2):
        contours = []
        blurred_hsv = cv2.pyrMeanShiftFiltering(hsv, 15, 20)
        
        if color == "green":
            #green mask
            lower_green = numpy.array([40, 50, 50]) 
            upper_green = numpy.array([80, 255, 255])
            mask = cv2.inRange(blurred_hsv, lower_green, upper_green)
        elif color == "red":
            #red mask
            mask = threshold_hsv_360(100, 50, 10, 255, 255, 340, blurred_hsv) #20, 320
        else:
            lower_green = numpy.array([40, 50, 50]) 
            upper_green = numpy.array([80, 255, 255])
            green_mask = cv2.inRange(blurred_hsv, lower_green, upper_green)
            red_mask = threshold_hsv_360(100, 50, 10, 255, 255, 340, blurred_hsv) #20, 320
            mask = red_mask | green_mask

        h, w, d = blurred_hsv.shape
        search_top = 0
        search_bot = h
        search_left = 0
        search_right = w
        print search_top, search_bot, search_left, search_right
        if loc == 1:
            print "loc = 1"
            search_top = 4 * h / 5
            search_bot = h
        elif loc == 2:
            print "loc = 2"
            search_top = 2 * h / 5
            search_bot = h
        elif loc == 3:
            print "loc = 3"
            search_top = 5 * h / 7
            search_left = 1 * w / 4 
            search_right = 3 * w / 4
        print search_top, search_bot, search_left, search_right
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        mask[0:h, 0:search_left] = 0
        mask[0:h, search_right:w] = 0

        _,all_countours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print("length = ", len(all_countours))

        for item in all_countours:
            contour = Contour.Unidentified
            perimeter = cv2.arcLength(item, True)
            vertices = cv2.approxPolyDP(item, 0.04 * perimeter, True)

            if len(vertices) == 3:
                contour = Contour.Triangle
            elif len(vertices) == 4:
                contour = Contour.Rectangle
            else:
                contour = Contour.Circle
            if contour != Contour.Unidentified and cv2.contourArea(item) > 1000:
                print contour, cv2.contourArea(item)
                cv2.drawContours(hsv, [item], -1, (255,0,0), 6)
                contours.append(contour)
        cv2.imshow("blur_hsv", hsv)
        cv2.imshow(color + "mask", mask)
        cv2.moveWindow(color + "mask", 710, 0)
        cv2.waitKey(4)

        return contours

def threshold_hsv_360(s_min, v_min, h_max, s_max, v_max, h_min, hsv):
    lower_color_range_0 = numpy.array([0, s_min, v_min],dtype=float)
    upper_color_range_0 = numpy.array([h_max/2., s_max, v_max],dtype=float)
    lower_color_range_360 = numpy.array([h_min/2., s_min, v_min],dtype=float)
    upper_color_range_360 = numpy.array([360/2., s_max, v_max],dtype=float)
    mask0 = cv2.inRange(hsv, lower_color_range_0, upper_color_range_0)
    mask360 = cv2.inRange(hsv, lower_color_range_360, upper_color_range_360)
    mask = mask0 | mask360
    return mask


