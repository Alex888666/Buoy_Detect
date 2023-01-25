#!/usr/bin/env python
from std_msgs.msg import UInt8
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import  CvBridge
import numpy as np



class camara:
    def __init__(self): 
        rospy.init_node('Color_id')
        self.s1 = rospy.Subscriber('/zed2/zed_node/left/image_rect_color',Image,self.img_callback) #Recibe el video
        #self.s2 = rospy.Subscriber('/counter_uInt8',UInt8,self.counter_callback) #Recibe el timer
        self.mask_pub = rospy.Publisher('/imgview',Image ,queue_size=10)
        self.density_pub = rospy.Publisher('/perc_color', Float32, queue_size=10)
        self.xy_pub = rospy.Publisher('/center',Float32MultiArray , queue_size=10)
        rospy.on_shutdown(self.stop)
        self.img = None
        self.bridge = CvBridge()
        self.t1 = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.xlimit = 1280
        self.ylimit = 720

    
    def img_callback(self,img): 
        try:       

            self.img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except:
            rospy.loginfo("Failed to convert image to CV2")

    def timer_callback(self, tl): 
        self.red_msk()
        self.red_density()
        self.center_xy()

    def red_msk(self):
        try:
            hsv = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
        except:
            rospy.loginfo("Failed HSV")

        lower_red = np.array([160,50,50])
        upper_red = np.array([180,255,255])
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([70, 255,255])
        lower_blue = np.array([94, 80, 2])
        upper_blue = np.array([126, 255,255])
        lower_white = np.array([0, 0, 231])
        upper_white = np.array([180, 180, 255])
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        maskr = cv.inRange(hsv, lower_green, upper_green)

        inv_mask = cv.bitwise_not(maskr)

        eroder = cv.erode(inv_mask, (5, 5), iterations = 6)
        eroder2 = cv.erode(maskr, (5, 5), iterations = 6)

        self.dilater = cv.dilate(eroder, (5, 5),iterations = 6)
        self.dilater2 = cv.dilate(eroder2, (5, 5),iterations = 6)

    #720
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(self.dilater))

    def center_xy(self):
        msg = Float32MultiArray()
        Mr = cv.moments(self.dilater)

        msg.data = [int(Mr["m10"] / Mr["m00"]), int(Mr["m01"] / Mr["m00"])] if Mr["m00"] != 0 else [int(self.xlimit/2), int(self.ylimit/2)]

        self.xy_pub.publish(msg)
    
    def red_density(self):
        self.density_pub.publish(np.sum(self.dilater2)/(self.xlimit*self.ylimit*255))

    def stop(self):
        print('Killing Color id')
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    cc = camara()
    
    try:
        cc.run()
    except:
        pass