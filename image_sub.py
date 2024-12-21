#!/usr/bin/env python
import rclpy

from rclpy.node import Node

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *
import cv2
from sensor_msgs.msg import Image
import numpy as np
import math
import time
import matplotlib.pyplot as plt

def compute_all_circle_centers(x1, y1, x2, y2, dir1_x, dir1_y, dir2_x, dir2_y, radius):
    # 垂直方向向量
    perp_dir1_x, perp_dir1_y = -dir1_y, dir1_x
    perp_dir2_x, perp_dir2_y = -dir2_y, dir2_x

    # 四组候选点的参数方程
    candidates = [
        (x1 + radius * perp_dir1_x, y1 + radius * perp_dir1_y, x2 + radius * perp_dir2_x, y2 + radius * perp_dir2_y),
        (x1 - radius * perp_dir1_x, y1 - radius * perp_dir1_y, x2 + radius * perp_dir2_x, y2 + radius * perp_dir2_y),
        (x1 + radius * perp_dir1_x, y1 + radius * perp_dir1_y, x2 - radius * perp_dir2_x, y2 - radius * perp_dir2_y),
        (x1 - radius * perp_dir1_x, y1 - radius * perp_dir1_y, x2 - radius * perp_dir2_x, y2 - radius * perp_dir2_y),
    ]
    print(f"candidates are {candidates}")
    circle_centers = []

    for p1_x, p1_y, p2_x, p2_y in candidates:
        # 解方程组求圆心
        A = [[dir1_x, -dir2_x], [dir1_y, -dir2_y]]
        b = [p2_x - p1_x, p2_y - p1_y]
        det = A[0][0] * A[1][1] - A[0][1] * A[1][0]

        if det == 0:
            # 如果平行，无解
            continue

        t1 = (b[0] * A[1][1] - b[1] * A[0][1]) / det
        center_x = p1_x + t1 * dir1_x
        center_y = p1_y + t1 * dir1_y
        print(f"center is {center_x}, {center_y}")
        # 添加结果
        circle_centers.append((center_x, center_y))

    return circle_centers

def angle_transform(angle):
    if angle > math.pi / 2:
        angle -= math.pi
    elif angle < -math.pi / 2:
        angle += math.pi
    else:
        angle = angle
    return angle


def find_tangent_point(h, k, x0, y0, dx, dy):
    a = dx**2 + dy**2
    b = 2 * (dx * (x0 - h) + dy * (y0 - k))
    t = -b / (2 * a)
    tangent_x = x0 + t * dx
    tangent_y = y0 + t * dy
    return tangent_x, tangent_y



def generate_tangent_arc_track(start, angle1, end, angle2, radius=40):
    x1, y1 = start
    x2, y2 = end

    angle1 = math.radians(angle_transform(angle1 + 90))
    angle2 = math.radians(angle_transform(angle2 + 90))

    # 计算起点和终点的方向向量
    dir1_x, dir1_y = math.cos(angle1), math.sin(angle1)
    dir2_x, dir2_y = math.cos(angle2), math.sin(angle2)

    # 计算所有可能的圆心
    circle_centers = compute_all_circle_centers(x1, y1, x2, y2, dir1_x, dir1_y, dir2_x, dir2_y, radius)
    # print(f"circles are {circle_centers}")

    # 选取到起点和终点距离总和最小的圆心
    best_center = None
    min_distance = float('inf')
    for center_x, center_y in circle_centers:
        dist = (math.sqrt((center_x - x1)**2 + (center_y - y1)**2) +
                math.sqrt((center_x - x2)**2 + (center_y - y2)**2))
        if dist < min_distance:
            min_distance = dist
            best_center = (center_x, center_y)
    print(best_center)

    circle_center_x, circle_center_y = best_center

    tangent1_x, tangent1_y = find_tangent_point(circle_center_x, circle_center_y, x1, y1, dir1_x, dir1_y)
    tangent2_x, tangent2_y = find_tangent_point(circle_center_x, circle_center_y, x2, y2, dir2_x, dir2_y)

    # 计算圆弧角度范围
    theta1 = math.atan2(tangent1_y - circle_center_y, tangent1_x - circle_center_x)
    theta2 = math.atan2(tangent2_y - circle_center_y, tangent2_x - circle_center_x)


    # 起点到切点1的直线
    line_step=25
    line1_length = math.sqrt((tangent1_x - x1)**2 + (tangent1_y - y1)**2)
    line1_points = max(1, int(line1_length // line_step))
    # line1_points = int(line1_length // line_step)
    line1_x = [x1 + (i + 1) * (tangent1_x - x1) / (line1_points) for i in range(line1_points - 1)]
    line1_y = [y1 + (i + 1) * (tangent1_y - y1) / (line1_points) for i in range(line1_points - 1)]
    print(f"line1_x: {line1_x}")
    print(f"line1_points: {line1_points - 1}, line_step 1 = {math.sqrt(((tangent1_x - x1) / line1_points)**2 + ((tangent1_y - y1) / line1_points)**2)}")
    # 切点2到终点的直线
    
    line2_length = math.sqrt((tangent2_x - x2)**2 + (tangent2_y - y2)**2)
    line2_points = max(1, int(line2_length // line_step))
    # line2_points = int(line2_length // line_step)
    line2_x = [x2 + (i + 1) * (tangent2_x - x2) / (line2_points) for i in range(line2_points - 1)]
    line2_y = [y2 + (i + 1) * (tangent2_y - y2) / (line2_points) for i in range(line2_points - 1)]
    line2_x.reverse
    line2_y.reverse
    # line2_x = [tangent2_x + i * (x2 - tangent2_x) / (line2_points) for i in range(line2_points)]
    # line2_y = [tangent2_y + i * (y2 - tangent2_y) / (line2_points) for i in range(line2_points)]
    print(f"line2_x: {line2_x}")
    print(f"line2_points: {line2_points - 1}, line_step 2 = {math.sqrt(((x2 - tangent2_x) / line2_points)**2 + ((y2 - tangent2_y) / line2_points)**2)}")

    # 按指定间隔生成圆弧上的点
    arc_step = 24
    arc_length = abs(theta2 - theta1) * radius
    arc_points = max(1, int(arc_length // arc_step))
    # arc_points = int(arc_length // arc_step)
    if theta2 -theta1 < 0:
        print(f"no inverse")
    else:
        print(f"inverse")
    arc_step = arc_length / arc_points
    theta_step = (theta2 - theta1) / (arc_points)  # 角度步长
    arc_points += 1
    t = [theta1 + i * theta_step for i in range(arc_points)]
    arc_x = [circle_center_x + radius * math.cos(theta) for theta in t]
    arc_y = [circle_center_y + radius * math.sin(theta) for theta in t]

    theta_step = (theta2 - theta1) / (arc_points + 1)
    t = [theta1 + (i + 1) * theta_step for i in range(arc_points)]
    # set t in degrees
    t = [math.degrees(theta) for theta in t]
    print(f"t: {t}, \n arc_points: {arc_points}, \n arc_x: {arc_x}, \n arc_step = {arc_step}")

    return line1_x, line1_y, line2_x, line2_y, arc_x, arc_y, t, best_center

def arc_track_placing(self, line1_x, line1_y, angle1, line2_x, line2_y, angle2, arc_x, arc_y, t, j):
    for i in range(len(line1_x)):
        targetP1 = f"{float(self.px_list2[j])}, {float(self.py_list2[j])}, 250, -180.00, 0.0, {float(self.angle_list2[j])-90}"
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        targetP2 = f"{float(self.px_list2[j])}, {float(self.py_list2[j])}, 134, -180.00, 0.0, {float(self.angle_list2[j])-90}"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        time.sleep(5)
        set_io(1.0)
        time.sleep(2)
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        # return origin
        targetP2 = "200, 380, 500, -180.00, 0.0, 135.00"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        targetP1 = f"{float(line1_x[i])}, {float(line1_y[i])}, 250, -180.00, 0.0, {angle1}"
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        targetP2 = f"{float(line1_x[i])}, {float(line1_y[i])}, 135, -180.00, 0.0, {angle1}"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        time.sleep(2)
        set_io(0.0)
        time.sleep(1)
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1) 
        # return origin
        targetP2 = "200, 380, 500, -180.00, 0.0, 135.00"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        j += 1   

    for i in range(len(arc_x)):
        targetP1 = f"{float(self.px_list2[j])}, {float(self.py_list2[j])}, 250, -180.00, 0.0, {float(self.angle_list2[j])-90}"
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        targetP2 = f"{float(self.px_list2[j])}, {float(self.py_list2[j])}, 134, -180.00, 0.0, {float(self.angle_list2[j])-90}"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        time.sleep(5)
        set_io(1.0)
        time.sleep(2)
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        # return origin
        targetP2 = "200, 380, 500, -180.00, 0.0, 135.00"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        targetP1 = f"{float(arc_x[i])}, {float(arc_y[i])}, 250, -180.00, 0.0, {float(t[i])}"
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        targetP2 = f"{float(arc_x[i])}, {float(arc_y[i])}, 135, -180.00, 0.0, {float(t[i])}"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        time.sleep(2)
        set_io(0.0)
        time.sleep(1)
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        # return origin
        targetP2 = "200, 380, 500, -180.00, 0.0, 135.00"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        j += 1

    for i in range(len(line2_x)):
        targetP1 = f"{float(self.px_list2[j])}, {float(self.py_list2[j])}, 250, -180.00, 0.0, {float(self.angle_list2[j])-90}"
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        targetP2 = f"{float(self.px_list2[j])}, {float(self.py_list2[j])}, 134, -180.00, 0.0, {float(self.angle_list2[j])-90}"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        time.sleep(5)
        set_io(1.0)
        time.sleep(2)
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        # return origin
        targetP2 = "200, 380, 500, -180.00, 0.0, 135.00"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        targetP1 = f"{float(line2_x[i])}, {float(line2_y[i])}, 250, -180.00, 0.0, {angle2}"
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        targetP2 = f"{float(line2_x[i])}, {float(line2_y[i])}, 135, -180.00, 0.0, {angle2}"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        time.sleep(2)
        set_io(0.0)
        time.sleep(1)
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        # return origin
        targetP2 = "200, 380, 500, -180.00, 0.0, 135.00"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        j += 1
    
    return j

def stair_placing(self, j):
    # stair
    for i in range(len(self.px_list_stair)):
        height_list = [133, 133, 147, 147, 168]
        targetP1 = f"{float(self.px_list2[j])}, {float(self.py_list2[j])}, 250, -180.00, 0.0, {float(self.angle_list2[j])-90}"
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        targetP2 = f"{float(self.px_list2[j])}, {float(self.py_list2[j])}, 134, -180.00, 0.0, {float(self.angle_list2[j])-90}"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        time.sleep(5)
        set_io(1.0)
        time.sleep(2)
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        # return origin
        targetP2 = "200, 380, 500, -180.00, 0.0, 135.00"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        targetP1 = f"{float(self.px_list_stair[i])},  {float(self.py_list_stair[i])},300, -180.00, 0.0, {float(self.angle_list_stair[i])-90}"
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        targetP2 = f"{float(self.px_list_stair[i])},  {float(self.py_list_stair[i])},{float(height_list[i])}, -180.00, 0.0, {float(self.angle_list_stair[i])-90}"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        time.sleep(2)
        set_io(0.0)
        time.sleep(1)
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
        # return origin
        targetP2 = "200, 380, 500, -180.00, 0.0, 135.00"
        script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        send_script(script2)
        j += 1
    
    return j

def euclidean_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)



class ImageSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Image, 'techman_image', self.image_callback, 10)
        self.is_first_image = True  # 用來標記是否為第一張影像
        self.px_list, self.py_list, self.angle_list = [], [], []
        self.px_list_stair, self.py_list_stair, self.angle_list_stair = [], [], []
        self.px_list2, self.py_list2, self.angle_list2 = [], [], []
    def image_callback(self, data):
        self.get_logger().info('Received image')
        if self.is_first_image:
            self.process_image1(data)  # 處理第一張影像
            self.is_first_image = False
        else:
            self.process_image2(data)  # 處理第二張影像
            self.is_first_image = True
            if self.px_list and self.px_list2:
                self.move()

    def process_image(self, data):
        img = np.array(data.data).reshape((data.height, data.width, 3))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(img, (5, 5), cv2.BORDER_DEFAULT)
        _, thresh = cv2.threshold(blur, 210, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours, img
    
    def process_image1(self, data):
        contours, img = self.process_image(data)
        self.px_list, self.py_list, self.angle_list, self.px_list_stair, self.py_list_stair, self.angle_list_stair = self.extract_features1(contours, img, is_first_image = True)

    def process_image2(self, data):
        contours, img = self.process_image(data)
        self.px_list2, self.py_list2, self.angle_list2 = self.extract_features2(contours, img, is_first_image=False)

    

    # deal with picture1 (path)    
    def extract_features1(self, contours, img, is_first_image):
        px_list, py_list, angle_list = [], [], []
        px_list_stair, py_list_stair, angle_list_stair = [], [], []
        for c in contours:
            M = cv2.moments(c)
            
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                area = cv2.contourArea(c)
                
                if area < 1000:  # 忽略過小的輪廓
                    continue

                rect = cv2.minAreaRect(c)
                width = float(rect[1][0])
                height = float(rect[1][1])
                angle = float(rect[2])
                area_true = width * height
                print (f"area is : {area_true}")

                if width < height:
                    angle = 90 - angle
                else:
                    angle = -angle


                # stair
                if area_true > 10000 and area_true < 35000:
                    if angle < 0:
                        angle3 = - angle
                    else:
                        angle3 = - angle

                    angle_rad = math.radians(angle3)
                    ux = math.cos(angle_rad)
                    uy = math.sin(angle_rad)
                    #offsets = [ -160, 160, -65, 68, 0]
                    offsets = [ -160, 160, -70, 70, 0]  
                    for offset in offsets:
                        new_x = cx + offset * ux
                        new_y = cy + offset * uy

                        if cx != 639 and cy != 479: # drop the center of the whole image
                            theta = math.radians(45)
                            r = 0.315
                            px = 200 - (new_y - 755)*r*math.sin(theta) + (new_x - 660)*r*math.sin(theta)
                            py = 380 - (new_x - 660)*r*math.cos(theta) - (new_y - 755)*r*math.cos(theta)
                            px_list_stair.append(px)
                            py_list_stair.append(py)
                            angle2 = 135.00 + angle
                            angle_list_stair.append(angle2)  
                # dominals
                elif area_true < 10000 :                              
                    if cx != 639 and cy != 479:
                        theta = math.radians(45)
                        r = 0.315
                        px = 200 - (cy - 755)*r*math.sin(theta) + (cx - 660)*r*math.sin(theta)
                        py = 380 - (cx - 660)*r*math.cos(theta) - (cy - 755)*r*math.cos(theta)
                        px_list.append(px)
                        py_list.append(py)
                        angle2 = 135.00 + angle
                        angle_list.append(angle2)
                else:
                    print ("do obstacle")
        return px_list, py_list, angle_list, px_list_stair, py_list_stair, angle_list_stair
    
    # deal with picture2 (holder)
    def extract_features2(self, contours, img,is_first_image):
        px_list, py_list, angle_list = [], [], []
        for c in contours:
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                area = cv2.contourArea(c)
                if area < 1000:  # 忽略過小的輪廓
                    continue

                rect = cv2.minAreaRect(c)
                angle = rect[2]
                if rect[1][0] < rect[1][1]:
                    angle = 90 - angle
                else:
                    angle = -angle
                if cx != 639 and cy != 479:
                    theta = math.radians(45)
                    r = 0.315
                    if is_first_image == 0  :
                        px = 410 - (cy - 755) * r*math.cos(theta) - (cx - 660) * r*math.cos(theta)
                        py = 200 - (cx - 660) * r*math.cos(theta) + (cy - 755) * r*math.cos(theta)
                    
                    px_list.append(px)
                    py_list.append(py)
                    angle_list.append(135.0 + angle)
                    print("No.particle")
        return px_list, py_list, angle_list
    

    def move(self):
        # 骨牌座標
        if len(self.px_list) == 1: # 骨牌1個
            a1 = (self.px_list[0],self.py_list[0])
            start = a1
            if len(self.px_list_stair) != 0: # 有階梯
                # 階梯兩端座標
                b1 = (self.px_list_stair[0],self.py_list_stair[0])
                b2 = (self.px_list_stair[1],self.py_list_stair[1])
                angle1 = self.angle_list[0]
                angle2 = self.angle_list_stair[1]+90
                # 計算 b1 與 a1 的距離和 b2 與 a1 的距離
                dist_b1_a1 = euclidean_distance(a1, b1)
                dist_b2_a1 = euclidean_distance(a1, b2)
                # 判斷誰比較近
                if dist_b1_a1 < dist_b2_a1:
                    end = b1
                else:
                    end = b2
                    self.px_list_stair[0], self.px_list_stair[1] = self.px_list_stair[1], self.px_list_stair[0]
                    self.py_list_stair[0], self.py_list_stair[1] = self.py_list_stair[1], self.py_list_stair[0]
                    self.px_list_stair[2], self.px_list_stair[3] = self.px_list_stair[3], self.px_list_stair[2]
                    self.py_list_stair[2], self.py_list_stair[3] = self.py_list_stair[3], self.py_list_stair[2]
                j = 0
                line1_x, line1_y, line2_x, line2_y, arc_x, arc_y, t, best_center = generate_tangent_arc_track(start, angle1, end, angle2)
                j = arc_track_placing(self, line1_x, line1_y, angle1, line2_x, line2_y, angle2, arc_x, arc_y, t, j)
                print("軌跡結束的j:", j)
                j = stair_placing(self, j)
                print("最終的j:", j)
            else:
                print("only one dominal, please put another")
        elif len(self.px_list) == 2: # 骨牌2個
            a1 = (self.px_list[0], self.py_list[0])
            a2 = (self.px_list[1], self.py_list[1]) 
            start = a1
            if len(self.px_list_stair) != 0: # 有階梯
                # 階梯兩端座標
                b1 = (self.px_list_stair[0], self.py_list_stair[0])
                b2 = (self.px_list_stair[1], self.py_list_stair[1])
                angle1 = self.angle_list[0]
                angle2 = self.angle_list_stair[1]+90
                # 計算 b1 與 a1 的距離和 b2 與 a1 的距離
                dist_b1_a1 = euclidean_distance(a1, b1)
                dist_b2_a1 = euclidean_distance(a1, b2)
                # 判斷誰比較近
                if dist_b1_a1 < dist_b2_a1:
                    end = b1
                    end2 = b2
                else:
                    end = b2
                    end2 = b1
                    self.px_list_stair[0], self.px_list_stair[1] = self.px_list_stair[1], self.px_list_stair[0]
                    self.py_list_stair[0], self.py_list_stair[1] = self.py_list_stair[1], self.py_list_stair[0]
                    self.px_list_stair[2], self.px_list_stair[3] = self.px_list_stair[3], self.px_list_stair[2]
                    self.py_list_stair[2], self.py_list_stair[3] = self.py_list_stair[3], self.py_list_stair[2]
                # 骨牌到階梯
                j = 0
                line1_x, line1_y, line2_x, line2_y, arc_x, arc_y, t, best_center = generate_tangent_arc_track(start, angle1, end, angle2)
                j = arc_track_placing(self, line1_x, line1_y, angle1, line2_x, line2_y, angle2, arc_x, arc_y, t, j)
                print("第一段的j:", j)
                j = stair_placing(self, j)
                print("階梯結束的j:", j)
                # 階梯到另一個骨牌
                start = end2
                end = a2
                angle1 = self.angle_list_stair[1]+90
                angle2 = self.angle_list[0]
                line1_x, line1_y, line2_x, line2_y, arc_x, arc_y, t, best_center = generate_tangent_arc_track(start, angle1, end, angle2)
                j = arc_track_placing(self, line1_x, line1_y, angle1, line2_x, line2_y, angle2, arc_x, arc_y, t, j)
                print("最終的j:", j)
            else: # 沒階梯，純排2個骨排
                angle1 = self.angle_list[0]
                angle2 = self.angle_list[1]
                end = a2
                j = 0
                line1_x, line1_y, line2_x, line2_y, arc_x, arc_y, t, best_center = generate_tangent_arc_track(start, angle1, end, angle2)
                j = arc_track_placing(self, line1_x, line1_y, angle1, line2_x, line2_y, angle2, arc_x, arc_y, t, j)
                print("最終的j:", j)

        elif len(self.px_list) == 3: # 骨牌3個
            a1 = (self.px_list[0], self.py_list[0])
            a2 = (self.px_list[1], self.py_list[1])
            a3 = (self.px_list[2], self.py_list[2])
            dist_a = euclidean_distance(a1, a2)
            dist_b = euclidean_distance(a1, a3)
            dist_c = euclidean_distance(a2, a3)
            if dist_a > dist_b and dist_a > dist_c:
                start = a1
                end = a3
                angle1 = self.angle_list[0]
                angle2 = self.angle_list[2]
                j = 0
                line1_x, line1_y, line2_x, line2_y, arc_x, arc_y, t, best_center = generate_tangent_arc_track(start, angle1, end, angle2)
                j = arc_track_placing(self, line1_x, line1_y, angle1, line2_x, line2_y, angle2, arc_x, arc_y, t, j)
                print("第一段的j:", j)
                # start = a3
                # end = a2
                # angle1 = self.angle_list[2]
                # angle2 = self.angle_list[1]
                start = a2
                end = a3
                angle1 = self.angle_list[1]
                angle2 = self.angle_list[0]
                line1_x, line1_y, line2_x, line2_y, arc_x, arc_y, t, best_center = generate_tangent_arc_track(start, angle1, end, angle2)
                j = arc_track_placing(self, line1_x, line1_y, angle1, line2_x, line2_y, angle2, arc_x, arc_y, t, j)
                print("最終的j:", j)
            elif dist_b > dist_a and dist_b > dist_c:
                start = a1
                end = a2
                angle1 = self.angle_list[0]
                angle2 = self.angle_list[1]
                j = 0
                line1_x, line1_y, line2_x, line2_y, arc_x, arc_y, t, best_center = generate_tangent_arc_track(start, angle1, end, angle2)
                j = arc_track_placing(self, line1_x, line1_y, angle1, line2_x, line2_y, angle2, arc_x, arc_y, t, j)
                print("第一段的j:", j)
                # start = a2
                # end = a3
                # angle1 = self.angle_list[1]
                # angle2 = self.angle_list[2]
                start = a3
                end = a2
                angle1 = self.angle_list[2]
                angle2 = self.angle_list[1]
                line1_x, line1_y, line2_x, line2_y, arc_x, arc_y, t, best_center = generate_tangent_arc_track(start, angle1, end, angle2)
                j = arc_track_placing(self, line1_x, line1_y, angle1, line2_x, line2_y, angle2, arc_x, arc_y, t, j)
                print("最終的j:", j)
            elif dist_c > dist_a and dist_c > dist_b:
                start = a2
                end = a1
                angle1 = self.angle_list[1]
                angle2 = self.angle_list[0]
                j = 0
                line1_x, line1_y, line2_x, line2_y, arc_x, arc_y, t, best_center = generate_tangent_arc_track(start, angle1, end, angle2)
                j = arc_track_placing(self, line1_x, line1_y, angle1, line2_x, line2_y, angle2, arc_x, arc_y, t, j)
                print("第一段的j:", j)
                # start = a1
                # end = a3
                # angle1 = self.angle_list[0]
                # angle2 = self.angle_list[2]
                start = a3
                end = a1
                angle1 = self.angle_list[2]
                angle2 = self.angle_list[0]
                line1_x, line1_y, line2_x, line2_y, arc_x, arc_y, t, best_center = generate_tangent_arc_track(start, angle1, end, angle2)
                j = arc_track_placing(self, line1_x, line1_y, angle1, line2_x, line2_y, angle2, arc_x, arc_y, t, j)
                print("最終的j:", j)
            else:
                print("something wrong")
        else:
            print("Number of dominals not 1,2,3")    
        plt.figure(figsize=(8, 8))
        plt.scatter(start[0], start[1], color='red', label="Start")
        plt.scatter(end[0], end[1], color='green', label="End")
        plt.scatter(line1_x, line1_y, label="Smooth Tangent Track 1", color='blue')
        plt.scatter(line2_x, line2_y, label="Smooth Tangent Track 2", color='blue')
        plt.scatter(arc_x, arc_y, label="Smooth Arc Track", color='orange')
        plt.scatter(best_center[0], best_center[1], color='purple', label="Chosen Circle Center")
        plt.legend()
        plt.grid()
        plt.axis('equal')
        plt.show()

        # # 拿holder的骨牌到Picture1對應的骨牌位置
        # for i in range(len(self.px_list1)):
        #     targetP1 = f"{float(self.px_list2[i])}, {float(self.py_list2[i])}, 165, -180.00, 0.0, {float(self.angle_list2[i])-90}"
        #     script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        #     send_script(script1)
        #     targetP2 = f"{float(self.px_list2[i])}, {float(self.py_list2[i])}, 165, -180.00, 0.0, {float(self.angle_list2[i])-90}"
        #     script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        #     send_script(script2)
        #     time.sleep(8)
        #     set_io(1.0)
        #     time.sleep(3)
        #     script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        #     send_script(script1)
        #     targetP2 = "300, 300, 500, -180.00, 0.0, 90.00"
        #     script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        #     send_script(script2)
        #     put = h*i + 115
        #     targetP2 = f"{float(self.px_list1[i])}, {float(self.py_list1[i])}, 165, -180.00, 0.0, {float(self.angle_list1[i])}"
        #     #targetP2 = f"300, 300, {put}, -180.00, 0.0, 90.00"
        #     script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        #     send_script(script2)
        #     time.sleep(8)
        #     set_io(0.0)
        #     targetP2 = "300, 300, 500, -180.00, 0.0, 90.00"
        #     script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        #     send_script(script2)


def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not availabe, waiting again...')
    
    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    gripper_node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSub('image_sub')
    
    rclpy.spin(node)
    

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

