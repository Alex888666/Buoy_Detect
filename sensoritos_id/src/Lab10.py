#!/usr/bin/env python
from std_msgs.msg import UInt8
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D, Twist
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import  CvBridge
import numpy as np

class camara:
    def __init__(self):
        self.dt = 0.1 #Time for interrupcion
        self.max_v = 0.11 #Velocidad linel maxima
        self.k_w = 0.155 #constante proporcional
        #0.15
        rospy.init_node('Color_id')

        self.s1 = rospy.Subscriber('/video_source/raw',Image,self.img_callback) #Recibe el video
        self.vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.mask_pub = rospy.Publisher('/imgview', Image ,queue_size=10)
        self.color_pub = rospy.Publisher('/imgviewcolor', Image ,queue_size=10)
        self.density_pub = rospy.Publisher('/perc_color', Float32, queue_size=10)
        self.flag_pub = rospy.Publisher('/flag', Bool, queue_size=10)
        rospy.Subscriber('/flag', Bool, self.flag_msg)

        self.img = None
        self.bridge = CvBridge()
        self.t1 = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        self.rate = rospy.Rate(10)
        self.xlimit = 1280
        self.ylimit = 720
        self.x1 = 0
        self.x2 = 0
        self.y1 = 0
        self.y2 = 0
        rospy.on_shutdown(self.stop)
        # self.color_pub.publish(self.bridge.cv2_to_imgmsg(self.dilater))

        self.flag = Bool()
        self.flag.data = False 

    def img_callback(self,msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def timer_callback(self, t):
        try:
            self.lines_msk()
            self.img = cv.resize(self.img,None, fx = .5, fy = .5)
        except:
            pass
        # self.red_density()
        # self.center_xy()

    def move_robot(self, linear, angular):
        msg = Twist() #Aqui se publica la velocidad lineal y angular
        msg.linear.x = linear
        msg.angular.z = self.k_w * angular
        self.vel.publish(msg)
    
    def color_msk(self, color):
        img_copy = self.img.copy()
        # cropped = img_copy[40:1240, 60: 660]
        cropped = img_copy[40:1240, 0: 360]
        hsv = cv.cvtColor(cropped, cv.COLOR_BGR2HSV)
        
        #lower_red = np.array([160,50,50])
        lower_red = np.array([160,110, 110])
        upper_red = np.array([180,255,255])
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([70, 255,255])
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        if color == 'r':
            maskr = cv.inRange(hsv, lower_red, upper_red)
        elif color == 'g':
            maskr = cv.inRange(hsv, lower_green, upper_green)
        elif color == 'y':
            maskr = cv.inRange(hsv, lower_yellow, upper_yellow)

        inv_mask = cv.bitwise_not(maskr)

        eroder = cv.erode(inv_mask, (5, 5), iterations = 6)
        eroder2 = cv.erode(maskr, (5, 5), iterations = 6)

        self.dilater = cv.dilate(eroder, (5, 5),iterations = 6)
        self.dilater2 = cv.dilate(eroder2, (5, 5),iterations = 6)

        self.color_pub.publish(self.bridge.cv2_to_imgmsg(self.dilater, "mono8"))
        density = np.sum(self.dilater2)/(self.xlimit*self.ylimit*255)
        density = density*10000
        self.density_pub.publish(density)

        return density
    
    # def center_xy(self):
    #     msg = Float32MultiArray()
    #     Mr = cv.moments(self.dilater)

    #     msg.data = [int(Mr["m10"] / Mr["m00"]), int(Mr["m01"] / Mr["m00"])] if Mr["m00"] != 0 else [int(self.xlimit/2), int(self.ylimit/2)]

    #     self.xy_pub.publish(msg)

    def lines_msk(self):
        cropped = self.img[int(self.ylimit*.75) : self.ylimit , 260 : 960]
        #alto = 180, largo = 700
        img_cen = np.array([350,180])
        gray = cv.cvtColor(cropped, cv.COLOR_BGR2GRAY)
        gg = cv.GaussianBlur(gray, (3,3), cv.BORDER_DEFAULT)
        edges = cv.Canny(gg,100,170,apertureSize=3)
        cv.circle(edges,(img_cen[0],img_cen[1]),2,(255,255,255),1)

        lines = cv.HoughLinesP(edges,1,np.pi/180,60,minLineLength=10,maxLineGap=10)
        #45


        try:
            self.x2 = 0
            self.x1 = 0
            self.y2 = 0
            self.y1 = 0
            for line in lines:
                x1,y1,x2,y2 = line[0]
                d = np.sqrt((x2-x1)**2+(y2-y1)**2)
                m = np.arctan2((y2-y1), (x2-x1))
                if m > np.pi/8 and m < np.pi*7/8 and d > 0.625:
                #1/8 7/8
                    if d > np.sqrt((self.x2-self.x1)**2+(self.y2-self.y1)**2):
                        self.x1 = x1
                        self.x2 = x2
                        self.y1 = y1
                        self.y2 = y2
            if self.x2 + self.y2 != 0:
                cv.line(edges,(self.x1,self.y1),(self.x2, self.y2),(255,0,0),2)
                #cv.line(self.img,(self.x1,self.y1),(self.x2, self.y2),(255,0,0),2)
                error = 1 - self.x2/350
                self.move_robot(self.max_v, error)
        except:
            self.move_robot(0.18,0.0)
            densities = []
            r_d = self.color_msk("r")
            densities.append(r_d)
            g_d = self.color_msk("g")
            densities.append(g_d)
            y_d = self.color_msk("y")
            densities.append(y_d)
            
            max_dens = max(densities)
            self.move_robot(0.0,0.0)
            if max_dens > 0.03:  #0.03
                max_index = densities.index(max_dens)
                if max_index == 0:
                    print('red')
                    self.move_robot(0.0, 0.0)
                elif max_index == 1:
                    self.move_robot(0.1, 0.0)
                else:
                    print('yellow')
            #pass

        # try:
        #     mag = []
        #     for line in lines:
        #         x1,y1,x2,y2 = line[0]
        #         mag.append(np.sqrt((x2-x1)**2+(y2-y1)**2))
        #         m = np.arctan2((y2-y1), (x2-x1))
        #         if m > np.pi/8 and m < np.pi*7/8:
        #             cv.line(edges,(x1,y1),(x2,y2),(255,0,0),2)
        #             cv.line(self.img,(x1, y1),(x2, y2),(255,0,0),2)
        #             #self.move_robot(self.max_v, 0.0)
        # except:
        #     #self.move_robot(0.0,0.0)
        #     pass

        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(edges, "mono8"))

    def red_density(self):
        self.density_pub.publish(np.sum(self.dilater2)/(self.xlimit*self.ylimit*255))

    def stop(self):
        print('Killing Color id')

    def flag_msg(self, msg):
        self.flag.data = msg.data

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    cc = camara()

    try:
        cc.run()
    except:
        pass