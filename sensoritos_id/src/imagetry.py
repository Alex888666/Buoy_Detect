#Codigo creado por Alejandro Hernández De la Torre
#Contacto :   Correo:alexhdlt@hotmail.com // Telefono: 5527443289 // Pueden buscarme en el grupo de wa de Vanttec como Alex Hernández // Soy de CCM por lo que no podre brindar ayuda en fisico
#La finalidad de este codigo es poder facilitar el uso de las camaras ZED2, ZED2i y la Realsense a cualquiera del equipo que las necesite.
#Tambien se da unos ejemplos de como usar estas camaras con OPENCV, algunos "trucos" y tambien compatibilidad con nodos de ROS, así como YOLO. 
#Aclaraciones: Este codigo ha sido probado en Ubuntu 18 y 19, por ende tambien en ROS Melodic y Noetic. En ambas plataformas corre bien. 
#Requisitos: 
# - Tener alguna versión de los sistemas operativos anteriormente mencionados  con su respectiva distribución de ROS.
# - Tener los paquetes instalados de la ZED o la Realsense
# - Incluir este archivo y compilarlo en su WorkSpace.
# - Tener OPENCV
# - Recomiendo Tener CUDA
#Cada parte del archivo con lo que hace se ira incluyendo en comentarios para que sea lo más claro posible. 




#La primera parte de este programa importa todos los componentes necesarios para poder mandar mensajes en ros, así como el correcto funcionamiento de todo los componentes de ROS, 
#tambien le dice exactamente donde debe de encontrar la distribución de python a usar.

#! /usr/bin/env python
import rospy
import ros_numpy
from std_msgs.msg import String, UInt8
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np

#Primero tenemos que declarar nuestra clase para poder llegar a hacer callbacks de los diferentes componentes que necesitamos para hacer funcionar en su totalidad los programas.

#Tenemos que iniciar con nuestro nodo, eso hacemos en la siguientes lineas despues de la definición de la clase

class Saver:
    def __init__(self):
        rospy.init_node('image_changer')


        
        #Como saben primero hay que definir a donde nos estamos suscribiendo, suscribir significa de donde estamos sacando la información para usarla nosotros, en este caso hay que 
        #dejar comentado los nodos a los que no nos vamos a suscribir dependiendo del caso y de que publishers tengamos activos en nuestro master de ROS.
        
        #Subscribers

#------------------------------------------------------------------------------------------------
        #Nodos ZED2

        self.s1 = rospy.Subscriber('/zed2/zed_node/rgb_raw/image_raw_color', Image, self.img_callback)
        #self.s1 = rospy.Subscriber('/zed2i/zed_node/rgb/image_rect_gray', Image, self.img2_callback)
#------------------------------------------------------------------------------------------------
        #Nodos ZED2I

        #self.s1 = rospy.Subscriber(/zed2i/zed_node/rgb/image_rect_gray, Image, self.img_callback)
#------------------------------------------------------------------------------------------------
        #Nodos Realsense
        # self.s1 = rospy.Subscriber('/camera/color/image_raw', Image, self.img_callback)
        # self.s2 = rospy.Subscriber('/camera/color/image_raw', Image, self.img2_callback)
#------------------------------------------------------------------------------------------------
        #Nodos YOLO V5
        #self.s3 = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.bb_callback)
        # self.scrop = rospy.Subscriber('/filter_img', Image, self.crop_callback)
#------------------------------------------------------------------------------------------------


        #Despues publicamos nuestros nodos es decir a donde queremos  


        #Publishers
        # self.pg = rospy.Publisher('Verde',Image, queue_size=10)
        # self.pr = rospy.Publisher('Rojo',Image, queue_size=10)
        # self.py = rospy.Publisher('Amarillo',Image, queue_size=10)
        # self.pb = rospy.Publisher('Azul',Image, queue_size=10)
        # self.pw = rospy.Publisher('Blanco',Image, queue_size=10)
        self.cl = rospy.Publisher('Limpio',Image, queue_size= 10)
        # self.lp = rospy.Publisher('Contornos',Image, queue_size=10)
        self.f = rospy.Publisher('Pruebas',Image, queue_size=10)
        # self.c = rospy.Publisher('circles',Image,queue_size=10)
        #self.yc = rospy.Publisher('filter_img', Image, queue_size=10)

        #Self datos
        self.img = None
        self.img2 = None
        self.xlimit = 1280
        self.ylimit = 720
        self.bridge = CvBridge()

        self.run()

    # def crop_callback(self, scrop):


    #     img_copy = ros_numpy.numpify(scrop.image)
    #     color = 'g'

    #     for box in scrop.bounds:
    #         cropped = img_copy[box.ymin:box.ymax, box.xmin:box.xmax]
        
    #     # #Detector de mayor densidad

    #         lower_green = np.array([36, 40, 40])
    #         upper_green = np.array([160,50,50])
    #         lower_red = np.array([160,110, 110])
    #         upper_red = np.array([180,255,255])
    #         lower_yellow = np.array([20, 100, 100])
    #         upper_yellow = np.array([30, 255, 255])
    #         lower_blue = np.array([94, 80, 2])
    #         upper_blue = np.array([115, 255,255])

    #         if color == 'g':
    #             maskm = cv.inRange(cropped, lower_green, upper_green)
    #         elif color == 'r':
    #             maskm = cv.inRange(cropped, lower_red, upper_red)
    #         elif color == 'y':
    #             maskm = cv.inRange(cropped, lower_yellow, upper_yellow)
    #         elif color == 'b':
    #             maskm = cv.inRange(cropped, lower_blue, upper_blue)

    #         erodem = cv.erode(maskm, (5,5), iterations= 6)
    #         dilatem = cv.dilate(erodem (5, 5),iterations = 6)
    #         density = np.sum(dilatem)/(self.xlimit*self.ylimit*255)
    #         density = density*10000
    #         cv.putText(self.dilater, str(density), (60, 230),cv.FONT_HERSHEY_TRIPLEX, 1.0, (0,0,0), 1)

    #         if color == 'g':
    #             cv.putText(self.dilater, 'green', (60, 330),cv.FONT_HERSHEY_TRIPLEX, 1.0, (0,0,0), 1)
    #         elif color == 'r':
    #             cv.putText(self.dilater, 'red', (60, 330),cv.FONT_HERSHEY_TRIPLEX, 1.0, (0,0,0), 1)
    #         elif color == 'y':
    #             cv.putText(self.dilater, 'yellow', (60, 330),cv.FONT_HERSHEY_TRIPLEX, 1.0, (0,0,0), 1)
    #         elif color == 'b':
    #             cv.putText(self.dilater, 'blue', (60, 330),cv.FONT_HERSHEY_TRIPLEX, 1.0, (0,0,0), 1)

    #         newImaged = Image()
    #         newImaged = newImaged= self.bridge.cv2_to_imgmsg(dilatem, "mono8")
    #         self.d.publish(newImaged)
    #         msg = color + ":" + str(density)
    #         self.densitytext.publish(msg)
    #         img_copy[box.ymin:box.ymax, box.xmin:box.xmax] = cropped



    #     self.d.publish = ros_numpy.numpify(img_copy)




    def img_callback(self, img):
        self.img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        cap = self.img
        hsv = cv.cvtColor(cap, cv.COLOR_BGR2HSV)
        xlimit = 1280
        ylimit = 720

        # #MASCARA VERDE
        # maskg = cv.inRange(hsv, (36, 40, 40), (86, 255,255))
        # dilateg = cv.dilate(maskg, (5, 5), iterations = 6)
        # erodeg = cv.erode(dilateg, (5, 5), iterations = 6)
        # deng = cv.countNonZero(erodeg)/(erodeg.size)
        # Mg = cv.moments(erodeg)
        # ceng = [int(Mg["m10"] / Mg["m00"]), int(Mg["m01"] / Mg["m00"])] if Mg["m00"] != 0 else [int(xlimit/2), int(ylimit/2)]
        # font = cv.FONT_HERSHEY_SIMPLEX
        # org = (20, 30)
        # org1 = (20, 60)
        # fontScale = 1
        # color = (127)
        # thickness = 1
        # erodeg = cv.putText(erodeg, str(round(deng*100))+"%", org, font, fontScale, color, thickness, cv.LINE_AA)
        # erodeg = cv.putText(erodeg, str(ceng), org1, font, fontScale, color, thickness, cv.LINE_AA)
        # notg=cv.bitwise_not(erodeg)
        # newImage=Image()
        # newImage = newImage= self.bridge.cv2_to_imgmsg(erodeg, "mono8")
        # self.pg.publish(newImage)

        # #MASCARA ROJA
        # maskr = cv.inRange(hsv, (160,50,50), (180, 255,255))
        # dilater = cv.dilate(maskr, (5, 5), iterations = 6)
        # eroder = cv.erode(dilater, (5, 5), iterations = 6)
        # (H,W) = cap.shape[:2]
        # denr = cv.countNonZero(eroder)/(eroder.size)
        # Mr = cv.moments(eroder)
        # cenr = [int(Mr["m10"] / Mr["m00"]), int(Mr["m01"] / Mr["m00"])] if Mr["m00"] != 0 else [int(xlimit/2), int(ylimit/2)]
        # font = cv.FONT_HERSHEY_SIMPLEX
        # org = (20, 30)
        # org1 = (20, 60)
        # fontScale = 1
        # color = (127)
        # thickness = 1
        # eroder = cv.putText(eroder, str(round(denr*100))+"%", org, font, fontScale, color, thickness, cv.LINE_AA)
        # eroder = cv.putText(eroder, str(cenr), org1, font, fontScale, color, thickness, cv.LINE_AA)
        # cv.imshow('framey', cv.bitwise_not(eroder))
        # newImager=Image()
        # newImager = newImager= self.bridge.cv2_to_imgmsg(eroder, "mono8")
        # self.pr.publish(newImager)

        # #MASCARA AMARILLA
        # masky = cv.inRange(hsv, (20, 100, 100), (30, 255,255))
        # dilatey = cv.dilate(masky, (5, 5), iterations = 6)
        # erodey = cv.erode(dilatey, (5, 5), iterations = 6)
        # (H,W) = cap.shape[:2]
        # deny = cv.countNonZero(erodey)/(erodey.size)
        # My = cv.moments(erodey)
        # ceny = [int(My["m10"] / My["m00"]), int(My["m01"] / My["m00"])] if My["m00"] != 0 else [int(xlimit/2), int(ylimit/2)]
        # font = cv.FONT_HERSHEY_SIMPLEX
        # org = (20, 30)
        # org1 = (20, 60)
        # fontScale = 1
        # color = (127)
        # thickness = 1
        # erodey = cv.putText(erodey, str(round(deny*100))+"%", org, font, fontScale, color, thickness, cv.LINE_AA)
        # erodey = cv.putText(erodey, str(ceny), org1, font, fontScale, color, thickness, cv.LINE_AA)
        # newImagey=Image()
        # newImagey = newImagey= self.bridge.cv2_to_imgmsg(erodey, "mono8")
        # self.py.publish(newImagey)

        # #MASCARA AZUL
        # maskb = cv.inRange(hsv, (94, 80, 2), (115, 255,255))
        # dilateb = cv.dilate(maskb, (5, 5), iterations = 6)
        # erodeb = cv.erode(dilateb, (5, 5), iterations = 6)
        # (H,W) = cap.shape[:2]
        # denb = cv.countNonZero(erodeb)/(erodeb.size)
        # Mb = cv.moments(erodeb)
        # cenb = [int(Mb["m10"] / Mb["m00"]), int(Mb["m01"] / Mb["m00"])] if Mb["m00"] != 0 else [int(xlimit/2), int(ylimit/2)]
        # font = cv.FONT_HERSHEY_SIMPLEX
        # org = (20, 30)
        # org1 = (20, 60)
        # fontScale = 1
        # color = (127)
        # thickness = 1
        # erodeb = cv.putText(erodeb, str(round(denb*100))+"%", org, font, fontScale, color, thickness, cv.LINE_AA)
        # erodeb = cv.putText(erodeb, str(cenb), org1, font, fontScale, color, thickness, cv.LINE_AA)
        # newImageb=Image()
        # newImageb = newImageb= self.bridge.cv2_to_imgmsg(erodeb, "mono8")
        # self.pb.publish(newImageb)
        
        # #MASCARA Blanca
        # maskw = cv.inRange(hsv, (0, 0, 195), (255, 60, 255))
        # dilatew = cv.dilate(maskw, (5, 5), iterations = 6)
        # erodew = cv.erode(dilatew, (5, 5), iterations = 6)
        # (H,W) = cap.shape[:2]
        # denw = cv.countNonZero(erodew)/(erodew.size)
        # Mw = cv.moments(erodew)
        # cenw = [int(Mw["m10"] / Mw["m00"]), int(Mw["m01"] / Mw["m00"])] if Mw["m00"] != 0 else [int(xlimit/2), int(ylimit/2)]
        # font = cv.FONT_HERSHEY_SIMPLEX
        # org = (20, 30)
        # org1 = (20, 60)
        # fontScale = 1
        # color = (127)
        # thickness = 1
        # erodew = cv.putText(erodew, str(round(denw*100))+"%", org, font, fontScale, color, thickness, cv.LINE_AA)
        # rodew = cv.putText(erodew, str(cenw), org1, font, fontScale, color, thickness, cv.LINE_AA)
        # newImagew=Image()
        # newImagew = newImagew= self.bridge.cv2_to_imgmsg(erodew, "mono8")
        # self.pw.publish(newImagew)

        #Mascara Limpia
        newImagecl = Image()
        newImagecl = newImagecl= self.bridge.cv2_to_imgmsg(cap, "bgr8")
        self.cl.publish(newImagecl)

        #Mascara De Corte
    # def img2_callback(self,msg):
    #     img2 = self.bridge.imgmsg_to_cv2(msg,"bgr8")

    #     #Detector de mayor densidad
    # def color_msk(self, color):
    #     img_copy = self.img.copy()
    #     hsv = cv.cvtColor(newImageCr, cv.COLOR_BGR2HSV)
        
    #     #lower_red = np.array([160,50,50])
    #     lower_red = np.array([160,110, 110])
    #     upper_red = np.array([180,255,255])
    #     lower_green = np.array([40, 40, 40])
    #     upper_green = np.array([70, 255,255])
    #     lower_yellow = np.array([20, 100, 100])
    #     upper_yellow = np.array([30, 255, 255])

    #     if color == 'r':
    #         maskr = cv.inRange(hsv, lower_red, upper_red)
    #     elif color == 'g':
    #         maskr = cv.inRange(hsv, lower_green, upper_green)
    #     elif color == 'y':
    #         maskr = cv.inRange(hsv, lower_yellow, upper_yellow)

    #     inv_mask = cv.bitwise_not(maskr)

    #     eroder = cv.erode(inv_mask, (5, 5), iterations = 6)
    #     eroder2 = cv.erode(maskr, (5, 5), iterations = 6)

    #     self.dilater = cv.dilate(eroder, (5, 5),iterations = 6)
    #     self.dilater2 = cv.dilate(eroder2, (5, 5),iterations = 6)

    #     self.color_pub.publish(self.bridge.cv2_to_imgmsg(self.dilater, "mono8"))
    #     density = np.sum(self.dilater2)/(self.xlimit*self.ylimit*255)
    #     density = density*10000
    #     self.density_pub.publish(density)

    #     return density



#----------------DIFERENTE CALLBACK----------------------------

#--------------------------CONVERSIONES CON ZED EN BLANCO Y NEGRO--------------------------------------
    #     #Line Detection


    #     #Circles
    # def img2_callback(self,imgc):
    #     self.imgc = self.bridge.imgmsg_to_cv2(imgc, "bgr8")
    #     capc = self.imgc
    #     hsv = cv.cvtColor(capc, cv.COLOR_BGR2HSV)
    #     blur = cv.Canny(hsv,80,250)
    #     circles = cv.HoughCircles(blur, cv.HOUGH_GRADIENT, dp = 1.2, minDist = 7, param1=100, param2=40, minRadius=0, maxRadius=20)
    #     for i in circles[0,:]:
    #     # draw the outer circle
    #         cv.circle(capc,(int(i[0]), int(i[1])), int(i[2]), (0, 255, 0), 2)
    #     # draw the center of the circle
    #         cv.circle(capc, (int(i[0]), int(i[1])), 2, (0, 0, 255), 3)
    #     newImageC = Image()
    #     newImageC = newImageC = self.bridge.cv2_to_imgmsg(capc,"bgr8")
    #     self.c.publish(newImageC)

#------------------------------CONVERSIONES EN REAL SENSE (Se hace un apartado diferente ya que la realsense necesita primero una conversion a blanco y negro)---------------------

    #     # Line Detection
    # def img2_callback(self,img2):
    #     self.img2 = self.bridge.imgmsg_to_cv2(img2, "bgr8")
    #     cap2 = self.img2
    #     gray = cv.cvtColor(cap2, cv.COLOR_BGR2GRAY)
    #     (H,W) = cap2.shape[:2]
    #     gg = cv.GaussianBlur(gray, (3,3), cv.BORDER_DEFAULT)
    #     edges = cv.Canny(gg,100,170,apertureSize=3)
    #     cv.circle(edges,(int(W/2),int(H/2)),2,(255,255,255),1)
    #     lines = cv.HoughLinesP(edges,1,np.pi/180,60,minLineLength=10,maxLineGap=10)
    #     newImageln=Image()
    #     newImageln = newImageln = self.bridge.cv2_to_imgmsg(edges,"mono8")
    #     self.lp.publish(newImageln)

    #     #ORB
    #     orb = cv.ORB_create(20,2,8,20,0,4,40,20,20)
    #     kp = orb.detect(gray,None) 
    #     kp, des = orb.compute(gray, kp)
    #     conv = cv.drawKeypoints(gray, kp, None, color=(0,0,255))
    #     newImageF=Image()
    #     newImageF = newImageF = self.bridge.cv2_to_imgmsg(conv,"bgr8")
    #     self.f.publish(newImageF)

        #Circles
    # def img2_callback(self,imgc):
    #     self.imgc = self.bridge.imgmsg_to_cv2(imgc, "bgr8")
    #     capc = self.imgc
    #     hsv = cv.cvtColor(capc, cv.COLOR_BGR2HSV)
    #     blur = cv.Canny(hsv,80,250)
    #     circles = cv.HoughCircles(blur, cv.HOUGH_GRADIENT, dp = 1.2, minDist = 7, param1=100, param2=40, minRadius=0, maxRadius=20)
    #     for i in circles[0,:]:
    #     # draw the outer circle
    #         cv.circle(capc,(int(i[0]), int(i[1])), int(i[2]), (0, 255, 0), 2)
    #     # draw the center of the circle
    #         cv.circle(capc, (int(i[0]), int(i[1])), 2, (0, 0, 255), 3)
    #     newImageC = Image()
    #     newImageC = newImageC = self.bridge.cv2_to_imgmsg(capc,"bgr8")
    #     self.c.publish(newImageC)
        
    def run(self):
        rospy.spin()

    def stop(self):
        print('Killing IMGTRY')


if __name__ == '__main__':
    s = Saver()

