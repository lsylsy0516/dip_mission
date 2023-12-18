# -*- coding:utf-8 -*-

import cv2
import numpy as np

"""
功能：读取一张图片，显示出来，转化为HSV色彩空间
     并通过设置HSV阈值，实时显示
"""

# get the frame of 3.mp4
cap = cv2.VideoCapture(0)
while True:
    ret , frame = cap.read()
    # 取左半边，frame尺寸未知
    
    if ret == False:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    hsv_lower = np.array([0, 100, 100])
    hsv_upper = np.array([255, 255, 255])
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    print(mask.shape[0])
    cv2.imshow("mask", mask)
    if cv2.waitKey(1) == ord('q'):
        break
cap.release()