#!/usr/bin/env python

#imports
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import  *
from sensor_msgs.msg import *
from nav_msgs.msg import  *
import numpy as np
import cv2 as cv
from math import *
from cv_bridge import CvBridge, CvBridgeError
import random
global create
import time
import tf


#global vars setup:
publish_img = True
publish_path = True
create = True

#callback class made for ros image object - depth, raw depth and infra-red mat.
class my_image:
    def __init__(self):
        self.bridge = CvBridge()
        self.ir_sub = rospy.Subscriber('/astra_rgb', Image, self.ir_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image', Image, self.depth_callback)
        self.ir = None
        self.depth = None

    def ir_callback(self, data):
        try:
            # data.encoding = "mono16"
            self.ir = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        try:
            data.encoding = "mono16"
            self.depth = self.bridge.imgmsg_to_cv2(data, "mono16")
        except CvBridgeError as e:
            print(e)

#callback class for reciving the correct location of the robot
class odoms_callbacks():
    def __init__(self):
        self.odom = Odometry()
        self.odom_starter = Odometry()
        odom_quat=tf.transformations.quaternion_from_euler(0,0,0)
        self.odom_starter.pose.pose = Pose(Point(0,0,0),Quaternion(*odom_quat))

    def odom_starter_callback(self,odom):
        global create
        self.odom_starter = odom
        create = True

    def odom_callback(self,odom):
        self.odom =odom

# main function
def analyze():
    #bridge - image compression for ros: required for publishing and subscribing to Image topics
    bridge = CvBridge()


    #inits node
    rospy.init_node('ir_analyzer_test')
    np.set_printoptions(suppress=True)
    rate = rospy.Rate(20)

    #ros subscribers and publishers setup
    image = my_image()
    if publish_img:
        image_pub  = rospy.Publisher('/camera/ir/proc', Image, queue_size=30)
    if publish_path:
        path_pub  = rospy.Publisher('/reflective_tpae_path', Path, queue_size=30)
        my_odoms_callback = odoms_callbacks()
        rospy.Subscriber('/odom', Odometry, my_odoms_callback.odom_callback)
        rospy.Subscriber('/opencv/odom', Odometry, my_odoms_callback.odom_starter_callback)

    while not rospy.is_shutdown():
        depth = image.depth
        ir = image.ir

        if ir is not None: # and depth is not None:
            
            now=time.time()

            #takes only high hue points

            ir=cv.resize(ir,(640,360))
            range_ = cv.inRange(ir, 180, 255)
            colored = cv.cvtColor(range_, cv.COLOR_GRAY2RGB)
            # depth = cv.cvtColor(rangeD_, cv.COLOR_GRAY2RGB)

            #smoothing and bluring noise
            kernal = np.ones((1,1))
            # print "kernal:", time.time()-now
            # now=time.time()
            colored = cv.erode(colored, kernal, iterations = 3)
            # print "erode:", time.time()-now
            # now=time.time()
            colored = cv.dilate(colored, np.ones((1,1)), iterations = 3)
            # print "dilate:", time.time()-now
            # now=time.time()
            kernel = np.ones((7,7),np.float32)/49
            # print "kernel:", time.time()-now
            # now=time.time()
            colored = cv.filter2D(colored,-1,kernel)
            # print "filter2D:", time.time()-now
            # now=time.time()
            # colored = cv.bilateralFilter(colored,9,75,75)
            # print "bilateral:", time.time()-now
            # now=time.time()
            # colored = cv.erode(colored, np.ones((3,3)), iterations = 5)
            # print "erode:", time.time()-now
            now=time.time()

            # contours
            ret,thresh = cv.threshold(colored,20,1022,0)
            thresh = cv.cvtColor(thresh, cv.COLOR_RGB2GRAY)
            _, contours,hierarchy = cv.findContours(thresh, 1, 2)

            squares = []
            for cnt in contours:
                square, angle, check = get_square(cnt)
                if check:
                    x, y = get_center(square)
                    squares.append((square, angle, (x,y)))

            cop = []
            for c in range(len(contours) - 1):
                cnt1, cnt2 = contours[c], contours[c+1]
                sq1, a1, check1 = get_square(cnt1)
                sq2, a2, check2 = get_square(cnt2)
                err = 4.0
                a_check = abs(a1) - err < abs(a2) < abs(a1) + err
                if check1 and check2 and a_check:
                    cop.append((sq1, sq2))
                    c += 1

            for s1, s2 in cop:
                cv.drawContours(colored, [s1], 0, (0,0,255), 2)
                cv.drawContours(colored, [s2], 0, (0,0,255), 2)


            for cnt in contours:
                cv.drawContours(colored, [cnt], 0, (0,0,255), 2)

            try:
                if publish_img:
                    # print "published"
                    image_pub.publish(bridge.cv2_to_imgmsg(colored,'rgb8'))
                    # print " "
            except Exception as e:
                print('wrong format: ', str(e))
        rate.sleep()

#---f--kin-----l---it----------------------------------------------------------------------------------#
#----uc---g--bu-lsh------------------------------------------------------------------------------------#
#---------------------------------------------useful functions----------------------------------------#
#-----------------------------------------------------------------------------------------------------#

#returns the distance and the angle from a reflective tape target
def get_path(cnt1, cnt2):
    pass
def calc_dist_from_target(center_point, target_width, image_width):
    image_width = 640 #Pixels
    camera_rat = 60 #degrees
    target_original_size = 30 #cm
    angle_mult = 0.5
    try:
        deg_from_camera = ((float(center_point[0])/float(image_width))*float(camera_rat) - (camera_rat/2)) * angle_mult #degrees
        deg_from_camera = deg_from_camera * angle_mult
    except:
        deg_from_camera = 0.0001

    dis = ((5*pow(10,-5))*pow(target_width,2)) - (0.0248 * target_width) + 3.6823
    return dis, deg_from_camera

#checks if the two squares have a avalid distance between them
def squareCheck(s1, s2, d):
    l1, l2, r1, r2 = s1[0][1], s1[0][1] ,s2[0][1] ,s2[0][1]
    for p in s1:
        if p[1] < l1:
            l1 = p[1]
        if p[1] > r1:
            r1 = p[1]
    for p in s2:
        if p[1] < l2:
            l2 = p[1]
        if p[1] > r2:
            r2 = p[1]
    d1, d2 = r1 - l1, r2 - r1
    ev = (d1 + d2)/2
    return ev < d


#returns the distance between two points
def get_dis(p1,p2):
    return sqrt((p2[1] - p1[1])**2 + (p2[0] - p1[0])**2)

#returns the angle between two points
def get_angle(p1, p2):
    a = float(p1[0])
    b = float(p2[0])
    c = float(p1[1])
    d = float(p2[1])
    try:
        m = (d-c)/(b-a)
        return m
    except:
        return 0

#returns the idial square for the given contours and the angle (direction) of the mass
def get_mass_dir(cnt):
    rect = cv.minAreaRect(cnt)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    max = 0
    a = 0
    for i in range(3):
        dis = get_dis(box[i], box[i+1])
        if dis > max:
            max = dis
            a = get_angle(box[i], box[i+1])
    return box, a


#checks if the given contours are in range of 30 precent of the perfect found square
def get_square(cnt):
    min_area = 300
    area = cv.contourArea(cnt)
    area_size_check = area > min_area
    cnt_len = cv.arcLength(cnt, True)
    approx = cv.approxPolyDP(cnt, 0.02*cnt_len, True)
    square, angle = get_mass_dir(cnt)
    square_area = cv.contourArea(square)
    err_rate = 0.7
    try:
        area_diff = square_area/area
    except:
        area_diff = 0
    refer = (1 - err_rate < area_diff < 1 + err_rate) and area_size_check
    return square, angle, refer

#returns the center points of contours
def get_center(cnt):
    if len(cnt) > 0:
        try:
            sumX, sumY, c = 0, 0, 0
            for point in cnt:
                sumX += point[0][0]
                sumY += point[0][1]
                c += 1
            cX, cY = sumX/c, sumY/c
            return (cX, cY)
        except:
            sumX, sumY, c = 0, 0, 0
            for point in cnt:
                sumX += point[0]
                sumY += point[1]
                c += 1
            cX, cY = sumX/c, sumY/c
            return (cX, cY)
    else:
        return 0, 0

#returns the center point between two point
def get_center_point(p1, p2):
    cx = (p1[0] + p2[0]) / 2
    cy = (p1[1] + p2[1]) / 2
    return (cx,cy)

# makes sure that the distance between the couples is related to the area of the couples
def areas_check(area1, area2, dist):
    err = 2.8
    return 1/err <= (area1/area2) <= 1*err

#claculates the angle of the reflective tape
def calc_tapes_deg(cnt_left, cnt_right, angle_to_target):
    magic_num = 0.1
    top_left = top_right = (1024,0)
    btn_left = btn_right = (0,0)

    for i in range(3):
        if cnt_left[i][0] < top_left[0]:
            top_left = cnt_left[i]
        if cnt_left[i][0] > btn_left[0]:
            btn_left = cnt_left[i]
        if cnt_right[i][0] < top_right[0]:
            top_right = cnt_right[i]
        if cnt_right[i][0] > btn_right[0]:
            btn_right = cnt_right[i]

    right_dist = top_right[0] - btn_right[0]
    left_dist = top_left[0] - btn_left[0]
    diff = (float(right_dist) - float(left_dist)) * 7 - float(angle_to_target) * magic_num
    return diff



#cubic spline path creator
def cubic_spline(spoint,epoint,k):
    dx = spoint[0]
    dy = spoint[1]
    cx  = k * cos(radians(spoint[2]))
    cy =k * sin(radians(spoint[2]))
    bx = 3*epoint[0]- 3*spoint[0] - k * cos(radians(epoint[2])) - 2*k*cos(radians(spoint[2]))
    by = 3*epoint[1]- 3*spoint[1] - k* sin(radians(epoint[2])) - 2*k*sin(radians(spoint[2]))
    ax = k * cos(radians(epoint[2])) -  2*epoint[0]+ k * cos(radians(spoint[2])) + 2*spoint[0]
    ay = k * sin(radians(epoint[2])) -  2*epoint[1]+ k * sin(radians(spoint[2])) + 2*spoint[1]
    points = []
    for s in range(0,1001,1):
        point = [ax*pow((s/1000.0),3) + bx*pow((s/1000.0),2) + cx*(s/1000.0) + dx,ay*pow((s/1000.0),3) + by*pow((s/1000.0),2) + cy*(s/1000.0) + dy]
        points.append(point)
    return points

#converts points array to ros Path
def cubic_2_path(points,offsets=[0,0]):
    path = Path()
    poses = []
    for x,y in points:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "/map"
        pose.pose.position.x = x - offsets[0]
        pose.pose.position.y = y - offsets[1]
        pose.pose.position.z = 0.13
        poses.append(pose)
    path.poses = poses
    path.header.frame_id = "/map"
    path.header.stamp = rospy.Time.now()
    return path


#calling main function
if __name__ == '__main__':
    analyze()
