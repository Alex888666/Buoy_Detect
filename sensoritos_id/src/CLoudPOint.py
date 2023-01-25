#! /usr/bin/env python

import rospy
from std_msgs.msg import String, UInt8
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
import pcl 

class Saver:
    def __init__(self):
        rospy.init_node('image_saver')
        #Subscribers

        self.s3 = rospy.Subscriber('/zed/zed_node/rgb_raw/image_raw_color', Image, self.img_callback)
        self.s4 = rospy.Subscriber('/zed/zed_node/rgb_raw/image_raw_gray', Image, self.img2_callback)
        
        #Publishers
        self.pg = rospy.Publisher('Verde',Image, queue_size=10)
        self.pr = rospy.Publisher('Rojo',Image, queue_size=10)
        self.py = rospy.Publisher('Amarillo',Image, queue_size=10)
        self.pb = rospy.Publisher('Azul',Image, queue_size=10)
        self.pw = rospy.Publisher('Blanco',Image, queue_size=10)
        self.cl = rospy.Publisher('Limpio',Image, queue_size= 10)
        self.lp = rospy.Publisher('COntornos',Image, queue_size=10)

        #Self datos

        self.date = ''
        self.counter = '0'
        self.img = None
        self.dilater = None
        self.img2 = None

        self.bridge = CvBridge()

        self.run()

    def hour_callback(self, msg):
        self.date = str(msg.data).replace(':','_')

    def counter_callback(self, msg):
        self.counter = str(msg.data)

    def img_callback(self, img):
        self.img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        cap = self.img
        hsv = cv.cvtColor(cap, cv.COLOR_BGR2HSV)
        xlimit = 640
        ylimit = 480

        #MASCARA VERDE
        maskg = cv.inRange(hsv, (36, 40, 40), (86, 255,255))
        dilateg = cv.dilate(maskg, (5, 5), iterations = 6)
        erodeg = cv.erode(dilateg, (5, 5), iterations = 6)
        deng = cv.countNonZero(erodeg)/(erodeg.size)

        Mg = cv.moments(erodeg)
        ceng = [int(Mg["m10"] / Mg["m00"]), int(Mg["m01"] / Mg["m00"])] if Mg["m00"] != 0 else [int(xlimit/2), int(ylimit/2)]
        font = cv.FONT_HERSHEY_SIMPLEX
        org = (20, 30)
        org1 = (20, 60)
        fontScale = 1
        color = (127)
        thickness = 1
        cv.putText(deng, str(round(deng*100))+"%", org, font, fontScale, color, thickness, cv.LINE_AA) 
        erodeg = cv.putText(erodeg, str(round(deng*100))+"%", org, font, fontScale, color, thickness, cv.LINE_AA)
        erodeg = cv.putText(erodeg, str(ceng), org1, font, fontScale, color, thickness, cv.LINE_AA)
        newImage=Image()
        newImage = newImage= self.bridge.cv2_to_imgmsg(erodeg, "mono8")
        self.pg.publish(newImage)

        #MASCARA ROJA
        maskr = cv.inRange(hsv, (160,50,50), (180, 255,255))
        dilater = cv.dilate(maskr, (5, 5), iterations = 6)
        eroder = cv.erode(dilater, (5, 5), iterations = 6)
        (H,W) = cap.shape[:2]
        denr = cv.countNonZero(eroder)/(eroder.size)
        Mr = cv.moments(eroder)
        cenr = [int(Mr["m10"] / Mr["m00"]), int(Mr["m01"] / Mr["m00"])] if Mr["m00"] != 0 else [int(xlimit/2), int(ylimit/2)]
        font = cv.FONT_HERSHEY_SIMPLEX
        org = (20, 30)
        org1 = (20, 60)
        fontScale = 1
        color = (127)
        thickness = 1
        eroder = cv.putText(eroder, str(round(denr*100))+"%", org, font, fontScale, color, thickness, cv.LINE_AA)
        eroder = cv.putText(eroder, str(cenr), org1, font, fontScale, color, thickness, cv.LINE_AA)
        cv.imshow('framey', cv.bitwise_not(eroder))
        newImager=Image()
        newImager = newImager= self.bridge.cv2_to_imgmsg(eroder, "mono8")
        self.pr.publish(newImager)

        #MASCARA AMARILLA
        masky = cv.inRange(hsv, (20, 100, 100), (30, 255,255))
        dilatey = cv.dilate(masky, (5, 5), iterations = 6)
        erodey = cv.erode(dilatey, (5, 5), iterations = 6)
        (H,W) = cap.shape[:2]
        deny = cv.countNonZero(erodey)/(erodey.size)
        My = cv.moments(erodey)
        ceny = [int(My["m10"] / My["m00"]), int(My["m01"] / My["m00"])] if My["m00"] != 0 else [int(xlimit/2), int(ylimit/2)]
        font = cv.FONT_HERSHEY_SIMPLEX
        org = (20, 30)
        org1 = (20, 60)
        fontScale = 1
        color = (127)
        thickness = 1
        erodey = cv.putText(erodey, str(round(deny*100))+"%", org, font, fontScale, color, thickness, cv.LINE_AA)
        erodey = cv.putText(erodey, str(ceny), org1, font, fontScale, color, thickness, cv.LINE_AA)
        newImagey=Image()
        newImagey = newImagey= self.bridge.cv2_to_imgmsg(erodey, "mono8")
        self.py.publish(newImagey)

        #MASCARA AZUL
        maskb = cv.inRange(hsv, (94, 80, 2), (115, 255,255))
        dilateb = cv.dilate(maskb, (5, 5), iterations = 6)
        erodeb = cv.erode(dilateb, (5, 5), iterations = 6)
        (H,W) = cap.shape[:2]
        denb = cv.countNonZero(erodeb)/(erodeb.size)
        Mb = cv.moments(erodeb)
        cenb = [int(Mb["m10"] / Mb["m00"]), int(Mb["m01"] / Mb["m00"])] if Mb["m00"] != 0 else [int(xlimit/2), int(ylimit/2)]
        font = cv.FONT_HERSHEY_SIMPLEX
        org = (20, 30)
        org1 = (20, 60)
        fontScale = 1
        color = (127)
        thickness = 1
        erodeb = cv.putText(erodeb, str(round(denb*100))+"%", org, font, fontScale, color, thickness, cv.LINE_AA)
        erodeb = cv.putText(erodeb, str(cenb), org1, font, fontScale, color, thickness, cv.LINE_AA)
        newImageb=Image()
        newImageb = newImageb= self.bridge.cv2_to_imgmsg(erodeb, "mono8")
        self.pb.publish(newImageb)
        
        #MASCARA Blanca
        maskw = cv.inRange(hsv, (0, 0, 195), (255, 60, 255))
        dilatew = cv.dilate(maskw, (5, 5), iterations = 6)
        erodew = cv.erode(dilatew, (5, 5), iterations = 6)
        (H,W) = cap.shape[:2]
        denw = cv.countNonZero(erodew)/(erodew.size)
        Mw = cv.moments(erodew)
        cenw = [int(Mw["m10"] / Mw["m00"]), int(Mw["m01"] / Mw["m00"])] if Mw["m00"] != 0 else [int(xlimit/2), int(ylimit/2)]
        font = cv.FONT_HERSHEY_SIMPLEX
        org = (20, 30)
        org1 = (20, 60)
        fontScale = 1
        color = (127)
        thickness = 1
        erodew = cv.putText(erodew, str(round(denw*100))+"%", org, font, fontScale, color, thickness, cv.LINE_AA)
        rodew = cv.putText(erodew, str(cenw), org1, font, fontScale, color, thickness, cv.LINE_AA)
        newImagew=Image()
        newImagew = newImagew= self.bridge.cv2_to_imgmsg(erodew, "mono8")
        self.pw.publish(newImagew)

        #Mascara Limpia
        newImagecl = Image()
        newImagecl = newImagew= self.bridge.cv2_to_imgmsg(cap, "bgr8")
        self.cl.publish(newImagecl)


        #Line Detection
    def img2_callback(self,img2):
        self.img2 = self.bridge.imgmsg_to_cv2(img2, "bgr8")
        cap2 = self.img2
        (H,W) = cap2.shape[:2]
        gg = cv.GaussianBlur(cap2, (3,3), cv.BORDER_DEFAULT)
        edges = cv.Canny(gg,100,170,apertureSize=3)
        cv.circle(edges,(int(W/2),int(H/2)),2,(255,255,255),1)
        lines = cv.HoughLinesP(edges,1,np.pi/180,60,minLineLength=10,maxLineGap=10)
        newImageln=Image()
        newImageln = newImageln = self.bridge.cv2_to_imgmsg(edges,"mono8")
        self.lp.publish(newImageln)
    def run(self):
        rospy.spin()

    def stop(self):
        print('Killing IMGTRY')


if __name__ == '__main__':
    s = Saver()

