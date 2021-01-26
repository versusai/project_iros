#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
from copy import deepcopy
import time

# roslaunch uuv_control_cascaded_pid key_board_velocity.launch model_name:=rexrov
# roslaunch curl_project start_rexrov_pid.launch

class Environment():
    def __init__(self):
        # self.image_pub = rospy.Publisher('camera/rgb/image_raw',Image)

        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image,self.image_callback)

        self.save_i = 0
        self.save = True
        self.start = time.time()
        self.end = time.time()
        self.image_sub = rospy.Subscriber('/rexrov/rexrov/camera/camera_image', Image,self.image_callback)
        rospy.sleep(1)
        cv2.imwrite('/root/shared/Docker/sub.png', self.cv_image) 
        # self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        #Keys CTRL + c will stop script
        rospy.on_shutdown(self.shutdown)

    def image_callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Image window", self.cv_image)
            cv2.waitKey(3)
            self.end = time.time()
            if(self.end - self.start > 5):
                print('aqui')
                self.start = time.time()
                cv2.imwrite('/root/shared/Docker/images/'+ str(self.save_i) +'_sub.png', self.cv_image)
                self.save_i += 1

        except CvBridgeError as e:
            print(e)
    
    def shutdown(self):
        #you can stop turtlebot by publishing an empty Twist
        #message
        rospy.loginfo("Stopping TurtleBot")
        # self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)

    def udcp(self):
        # self.cv_image = cv2.imread('Test3.png')

        # cv2.imshow("Image window", self.cv_image)
        # cv2.waitKey(0)

        # self.image_original = self.cv_image

        # compute LUT
        lut = np.zeros(256)

        for i, _ in enumerate(lut):
            if i == 0:
                lut[i] = -math.log(0.1/255.)
            else:
                lut[i] = -math.log(float(i)/255)

        # print(lut)



        img_rgb = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
        # cv2.imshow("Image window 2", img_rgb)
        # cv2.waitKey(0)

        height, width, _ = self.cv_image.shape
        # print('aqui1', height, width)

        img_min = np.zeros((height, width, 1), np.uint8)
        # cv2.imshow("Image window 2", img_min)
        # cv2.waitKey(3)

        min_height = 100
        min_h, min_w = 0, 0
        highest = 0



        # Compute global light slow
        # for i in range(0, height):
        #     for j in range(0, width):
        #         img_min[i,j] = self.cv_image[i,j,1] if self.cv_image[i,j,0] > self.cv_image[i,j,1] else self.cv_image[i,j,0]
        #         if img_min[i,j]> highest and i < min_height:
        #             min_h = i
        #             min_w = j
        #             highest = img_min[i, j]

        for i in range(0, height):
            for j in range(0, width):
                value = self.cv_image.item(i,j,1) if self.cv_image.item(i,j,0) > self.cv_image.item(i,j,1) else self.cv_image.item(i,j,0)
                img_min.itemset((i,j, 0), value)
                if img_min.item(i,j, 0)> highest and i < min_height:
                    min_h = i
                    min_w = j
                    highest = img_min.item(i, j, 0)

        # cv2.imshow("Image window img_min", img_min)
        # cv2.waitKey(3)

        aux_a1 = self.cv_image[min_h, min_w, 0]
        aux_a2 = self.cv_image[min_h, min_w, 1] 
        aux_a3 = self.cv_image[min_h, min_w, 2] 

        # aux_a1 = 255.
        # aux_a2 = 255.
        # aux_a3 = 255. 

        print('min_h: ', min_h)
        print('min_w: ', min_w)
        print('highest: ', highest)
        print('aux_a1: ', aux_a1)
        print('aux_a2: ', aux_a2)
        print('aux_a3: ', aux_a3)

        b, g, r = cv2.split(deepcopy(self.cv_image))
        r1 = deepcopy(r*(255.0/aux_a1))
        print(r1.shape)
        r1 = r1.astype('uint8')
        print(r1.shape)
        g1 = deepcopy(g*(255.0/aux_a2))
        g1 = g1.astype('uint8')
        b1 = deepcopy(b*(255.0/aux_a3))
        b1 = b1.astype('uint8')

        # print(r1.dtype)
        # print(self.cv_image.dtype)


        # img_merged = cv2.merge((b1,g1,r1))
        # img_min = cv2.min(b1,g1)

        # print(img_merged)
        # cv2.imshow("Image window 4", img_merged)
        # cv2.waitKey(0)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(15,15))
        # print(kernel)


        img_udcp = cv2.erode(img_min, kernel, anchor=(7,7))

        for i in range(0, height):
            for j in range(0, width):
                value = int(255-0.95*img_udcp.item(i,j))
                img_udcp.itemset((i,j), value)

        # cv2.imshow("Image window img_udcp", img_udcp)
        # cv2.waitKey(0)

        img_depth = cv2.LUT(img_udcp, lut)
        img_depth = img_depth.astype('float32')
        img_depth = cv2.medianBlur(img_depth, 5)
        img_depth = cv2.GaussianBlur(img_depth, (11,11),0)

        max_depth = img_depth.max()



        # cv2.imshow("Image window img_depth", img_depth)
        # cv2.waitKey(3)
        # print('max',img_depth.max())

        img_binary = img_depth*(255./1.1)
        # img_binary = img_binary.astype('uint8')
        _, img_binary = cv2.threshold(img_binary, 10, 255, cv2.THRESH_BINARY)
        img_binary = img_binary.astype('uint8')

        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(100,100))
        img_binary = cv2.erode(img_binary, element, anchor=(45, 45))

        # img_binary = img_binary.astype('uint8')
        # print('dfw',img_binary.dtype)
        print('dfw',img_binary[0,1])

        # cv2.imshow("Image window img_binary", img_binary)
        # cv2.waitKey(3)

        sumzero = cv2.countNonZero(img_binary)
        print('sumzero',sumzero)

        #area rov -> radius 45 pixels
        area_rov = math.pi*45*45

        radius_rov = 45*2

        find_area = False

        if sumzero > area_rov:

            _, cont, _ = cv2.findContours(img_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # print('v',cont)
            # print(cont)
            img_drawed = cv2.drawContours(deepcopy(self.cv_image), cont, -1, (0,255,0), cv2.FILLED)

            max_index_area = 0
            area_temp = 0.
            for i, c in enumerate(cont):
                area = cv2.contourArea(c)
                if area_temp < area:
                    # print('oi')
                    max_index_area = i
                    area_temp = area
                print('aqui42: ', area)

            print('vai ',max_index_area)
            print('vai ',cv2.contourArea(cont[max_index_area]))

            img_1 = deepcopy(self.cv_image)

            for i, c in enumerate(cont):
                if i != max_index_area:
                    img_center = cv2.drawContours(img_binary, cont, i, 0, cv2.FILLED)
                else:
                    img_center = deepcopy(img_binary)

            # cv2.imshow("Image test", img_center)
            # cv2.waitKey(0)

            print('cont: ', len(cont))
            ctmassx = 0.0
            ctmassy = 0.0
            meandepth = 0.0
            pixels = 0

            if(len(cont)>0):
                # for i_c, c in enumerate(cont):
                for i in range(0, height):
                    for j in range(0, width):
                        # print('s', img_center.shape)
                        if(img_center.item(i,j) == 255):
                            area = cv2.contourArea(c)
                            # if(area > area_rov):
                            pixels += 1
                            ctmassx += j
                            ctmassy += i
                            meandepth += img_depth.item(i,j)

            ctmassx /= pixels
            ctmassy /= pixels
            meandepth = pixels

            

            print("aqui1: " + str(pixels) + ' ' + str(ctmassx) + ' ' + str(ctmassy) + ' ' + str(meandepth))
            print('aqui2: ', img_depth[0,0])


            # cv2.imshow("Image window img_drawed", img_drawed)
            # cv2.waitKey(0)

            img_depthc = cv2.cvtColor(img_depth, cv2.COLOR_GRAY2RGB)

            
            if (len(cont) == 1 and cont[0].shape[0] == 4):
                cv2.circle(img_drawed, (int(ctmassx), int(ctmassy)), radius_rov, (255,255,0), 5)
                cv2.circle(img_depthc, (int(ctmassx), int(ctmassy)), radius_rov, (255,255,0), 5)
                find_area = True
            else:
                ellipse = cv2.fitEllipse(cont[max_index_area])
                (x,y),(width_e,height_e),angle = cv2.fitEllipse(cont[max_index_area])
                print('aqui7: ' + str(width_e) + ' ' + str(height_e))

                if(radius_rov <= width_e and radius_rov <= height_e):
                    cv2.ellipse(img_drawed, ellipse, (0,255,255), 3)
                    cv2.circle(img_drawed, (int(ctmassx), int(ctmassy)), radius_rov, (255,255,0), 5)

                    cv2.ellipse(img_depthc, ellipse, (0,255,255), 3)
                    cv2.circle(img_depthc, (int(ctmassx), int(ctmassy)), radius_rov, (255,255,0), 5)
                    find_area = True

            # img_drawed = cv2.drawContours(img_drawed, cont[max_index_area], -1, (0,0,255), cv2.FILLED)

            # #end-------------------------------
            # end = time.time()

            # print('****\ntime: ', end - start)
            # print('****\n')
            # #end-------------------------------

            # cv2.imshow("Image window img_drawedc", img_drawed)
            # cv2.waitKey(0)

            print('aqui------------')

            cv2.imshow("Image window img_depthc", img_depthc)
            cv2.waitKey(3)
        


if __name__ == '__main__':
    rospy.init_node('env', anonymous=True)

    env = Environment()

    rospy.spin()

    # while True:
    #     try:
    #         print('aqui')
    #     except KeyboardInterrupt:
    #         print("Shutting down")
    #         cv2.destroyAllWindows()
    #         break

cv2.destroyAllWindows()