#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import math
import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *
import cv2
from sensor_msgs.msg import Image
from tm_msgs.srv import SendScript
from scipy.interpolate import splprep, splev
import numpy as np
import math
import time
import matplotlib.pyplot as plt
import heapq


class ImageSub(Node):
    def a_star(self, grid, start, end):
        """A* 演算法計算最短路徑"""
        rows, cols = grid.shape
        open_set = []
        heapq.heappush(open_set, (0, start))  # 優先級佇列
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, end)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == end:
                return self.reconstruct_path(came_from, current)

            neighbors = self.get_neighbors(current, rows, cols)
            for neighbor in neighbors:
                if grid[neighbor[0], neighbor[1]] == 1:  # 障礙物
                    continue
                tentative_g_score = g_score[current] + (1.414 if abs(neighbor[0] - current[0]) + abs(neighbor[1] - current[1]) == 2 else 1)

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, end)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # 如果無法到達，返回空路徑

    def heuristic(self, a, b):
        """計算啟發函數 (歐幾里得距離)"""
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def get_neighbors(self, pos, rows, cols):
        """獲取相鄰位置，包括斜向"""
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]:
            nx, ny = pos[0] + dx, pos[1] + dy
            if 0 <= nx < rows and 0 <= ny < cols:
                neighbors.append((nx, ny))
        return neighbors


    def reconstruct_path(self, came_from, current):
        """重建路徑"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(
            Image, 'techman_image', self.image_callback, 10
        )
        self.subscription
        self.is_first_image = True  # 用來標記是否為第一張影像
        self.px_list, self.py_list, self.angle_list = [], [], []
        self.px_list2, self.py_list2, self.angle_list2 = [], [], []
        self.cx_list_ob, self.cy_list_ob, self.c_angle_list_ob = [], [], []

    def image_callback(self, data):
        self.get_logger().info('Received image')
        if self.is_first_image:
            self.process_image1(data)  # 處理第一張影像 (map)
            self.is_first_image = False
        else:
            self.process_image2(data)  # 處理第二張影像 (holder)
            self.is_first_image = True
            if self.cx_list_ob and self.px_list2:
                print("check")
                self.move()  # 當兩張影像處理完成後執行移動操作

    def process_image(self, data):
        img = np.array(data.data).reshape((data.height, data.width, 3))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(img, (5, 5), cv2.BORDER_DEFAULT)
        _, thresh = cv2.threshold(blur, 210, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours, img

    def process_image1(self, data):
        self.cx_list_ob, self.cy_list_ob, self.c_angle_list_ob = self.extract_features1(data)

    def process_image2(self, data):
        contours, img = self.process_image(data)
        self.px_list2, self.py_list2, self.angle_list2 = self.extract_features2(contours, img, is_first_image=False)

    def extract_features1(self, data):
        self.get_logger().info('Received image')
        # 1. 從 ROS 訂閱影像並轉換為 NumPy 格式
        img = np.array(data.data)
        img = img.reshape((data.height, data.width, 3))

        # 保存原始影像
        original_img = img.copy()
        # 轉換到 HSV 色彩空間
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 辨識骨牌（白色）和障礙物（紅色）
        lower_white = np.array([0, 0, 230])
        upper_white = np.array([180, 30, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        lower_red1 = np.array([0, 150, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 150, 100])
        upper_red2 = np.array([180, 255, 255])
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # 膨脹障礙物
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (200, 200))  # 膨脹大小
        mask_red_dilated = cv2.dilate(mask_red, kernel)

        # 生成佔據地圖
        occupied_map = mask_red_dilated // 255  # 障礙物為 1，其他為 0

        # 找到骨牌中心點和寬邊方向
        contours_white, _ = cv2.findContours(mask_white, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        dominoes = []
        for contour in contours_white:
            area = cv2.contourArea(contour)
            if area < 1000:  # 忽略面積過小的輪廓
                continue

            # 計算骨牌的最小外接矩形
            rect = cv2.minAreaRect(contour)
            center = (int(rect[0][1]), int(rect[0][0]))  # 交換 X 和 Y 軸
            width, height = rect[1]
            angle = rect[2]

            # 確保寬邊是最長的邊
            if width < height:
                angle = 90 - angle
                width, height = height, width
            else:
                angle = -angle

            dominoes.append({'center': center, 'angle': angle, 'width': width, 'height': height})

        if len(dominoes) < 2:
            self.get_logger().info('Not enough dominoes detected!')
            return

        # 設定起點和終點
        start_domino = dominoes[0]
        start_center = start_domino['center']
        start_angle = start_domino['angle']
        start_offset = int(start_domino['width'] / 2)
        start = (
            int(start_center[0] + start_offset * np.cos(np.radians(start_angle))),
            int(start_center[1] + start_offset * np.sin(np.radians(start_angle)))
        )

        end_domino = dominoes[1]
        end_center = end_domino['center']
        end_angle = end_domino['angle']
        end_offset = int(end_domino['width'] / 2)
        end = (
            int(end_center[0] - end_offset * np.cos(np.radians(end_angle))),
            int(end_center[1] - end_offset * np.sin(np.radians(end_angle)))
        )

        # 2. 使用 A* 演算法尋找最短路徑
        path = self.a_star(occupied_map, start, end)

        # 繪製 A* 路徑到影像
        for point in path:
            cv2.circle(img, (point[1], point[0]), 2, (0, 255, 0), -1)  # 綠色路徑點

        # 保存 A* 路徑結果
        cv2.imwrite('a_star_path.png', img)

        # 3. 將路徑點分段記錄為 B-Spline 控制點
        control_points = path[::50]  # 每隔 10 點取一個控制點
        control_points = np.array(control_points)

        # 4. 使用 B-Spline 生成圓滑曲線
        tck, u = splprep([control_points[:, 1], control_points[:, 0]], s=20)
        u_fine = np.linspace(0, 1, 500)
        spline_x, spline_y = splev(u_fine, tck)

        # 繪製 B-Spline 曲線到影像
        for x, y in zip(spline_x, spline_y):
            cv2.circle(img, (int(x), int(y)), 1, (255, 0, 0), -1)  # 藍色曲線點

        # 保存 B-Spline 曲線結果
        cv2.imwrite('bspline_path.png', img)

        # 5. 計算最終骨牌放置點
        cx_list_ob = []
        cy_list_ob = []
        c_angle_list_ob = []
        for i in range(45, len(spline_x), 45):  # 每隔 100 點取放置點，忽略第一個點
            x, y = spline_x[i], spline_y[i]

            # 座標轉換
            theta = np.radians(45)
            r = 0.315
            px = 200 - (y - 755) * r * np.sin(theta) + (x - 660) * r * np.sin(theta)
            py = 380 - (x - 660) * r * np.cos(theta) - (y - 755) * r * np.cos(theta)

            cx_list_ob.append(px)
            cy_list_ob.append(py)

            # 在影像中繪製放置點
            cv2.circle(img, (int(x), int(y)), 5, (0, 0, 255), -1)  # 紅色點表示放置點

        for i in range(len(cx_list_ob)):
            if i < len(cx_list_ob) - 1:  # 不是最后一个点
                dx = cx_list_ob[i + 1] - cx_list_ob[i]
                dy = cy_list_ob[i + 1] - cy_list_ob[i]
            else:  # 最后一个点，取与倒数第二个点的方向
                dx = cx_list_ob[i] - cx_list_ob[i - 1]
                dy = cy_list_ob[i] - cy_list_ob[i - 1]

            # 计算方向角并调整为期望角度范围
            angle = np.degrees(np.arctan2(dy, dx))
            c_angle_list_ob.append(angle)




        # 移除最後一個放置點如果它距離終點過近
        if len(cx_list_ob) > 0:
            last_x, last_y = spline_x[-1], spline_y[-1]
            final_x, final_y = cx_list_ob[-1], cy_list_ob[-1]
            distance = np.sqrt((last_x - final_x) ** 2 + (last_y - final_y) ** 2)
            if distance < 50:
                cx_list_ob.pop()
                cy_list_ob.pop()
                c_angle_list_ob.pop()

        print("Final B-Spline Points:")
        print("X Coordinates:", cx_list_ob)
        print("Y Coordinates:", cy_list_ob)
        print("Angles:", c_angle_list_ob)
        # 保存最終放置點影像
        cv2.imwrite('final_placement_points.png', img)

        return cx_list_ob, cy_list_ob, c_angle_list_ob


    # deal with picture2 (holder)
    def extract_features2(self, contours, img, is_first_image):
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
        j = 0
        for i in range(len(self.cx_list_ob)):
            # 抓取骨牌 (holder)
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
            # 放置骨牌 (map)
            targetP1 = f"{float(self.cx_list_ob[i])},  {float(self.cy_list_ob[i])}, 250, -180.00, 0.0, {float(self.c_angle_list_ob[i])-90}"
            script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
            send_script(script1)
            targetP2 = f"{float(self.cx_list_ob[i])},  {float(self.cy_list_ob[i])}, 140, -180.00, 0.0, {float(self.c_angle_list_ob[i])-90}"
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

            j += 1  # 更新抓取點索引

def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not available, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        gripper_node.get_logger().info('service not available, waiting again...')

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
