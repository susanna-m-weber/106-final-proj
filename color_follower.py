import cv2
import numpy as np
from matplotlib import pyplot as plt
import serial 
import time

print("Opening camera, please wait ~ 1 minute")
cap = cv2.VideoCapture(0)
print(f"Camera ready: {cap.isOpened()}")

x_ball, y_ball, x_goalie, y_goalie = 0, 0, 0, 0

arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)

while cap.isOpened():
    _, imageFrame = cap.read()
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    
    blue_lower = np.array([90, 50, 70], np.uint8)
    blue_upper = np.array([115, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    
    green_lower = np.array([30, 100, 50], np.uint8) #np.array([60, 100, 50], np.uint8)
    green_upper = np.array([80, 255, 255], np.uint8) #np.array([80, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    kernal = np.ones((5, 5), "uint8")
    
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask = blue_mask)
    
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame, mask = green_mask)
    
    # Creating contour to track  color
    contours_b, hierarchy_b = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_g, hierarchy_g = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # contours_r, hierarchy_r = cv2.findContours(red_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
        
    if contours_g:
        green_max_contour = max(contours_g, key = cv2.contourArea)
        if cv2.contourArea(green_max_contour) > 50:
            x_g_max, y_g_max, w_g_max, h_g_max = cv2.boundingRect(green_max_contour)
            imageFrame = cv2.rectangle(imageFrame, (x_g_max, y_g_max), (x_g_max + w_g_max, y_g_max + h_g_max), (0, 255, 0), 2)
            cv2.putText(imageFrame, f"Center Coordinate: {(x_g_max + w_g_max) / 2}, {(y_g_max + h_g_max) / 2}", (x_g_max, y_g_max), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0))
            cv2.putText(imageFrame, f"Big green boy: x = {x_g_max}, y = {y_g_max}",(50, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))
            x_ball = (x_g_max + w_g_max) / 2
            y_ball = (y_g_max + h_g_max) / 2
            
            #D = 250
            #arduino.write(bytes("250", 'utf-8'))
            #time.sleep(0.05)
            #datag = arduino.readline()
            #print(datag)
    
    if contours_b:
        blue_max_contour = max(contours_b, key = cv2.contourArea)
        x_b_max, y_b_max, w_b_max, h_b_max = cv2.boundingRect(blue_max_contour)
        imageFrame = cv2.rectangle(imageFrame, (x_b_max, y_b_max), (x_b_max + w_b_max, y_b_max + h_b_max), (255, 0, 0), 2)
        cv2.putText(imageFrame, f"Center Coordinate: {(x_b_max + w_b_max) / 2}, {(y_b_max + h_b_max) / 2}", (x_b_max, y_b_max), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0))
        cv2.putText(imageFrame, f"Big blue boy: {x_b_max} {y_b_max}",(50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))
        x_goalie = (x_b_max + w_b_max) / 2
        y_goalie = (y_b_max + h_b_max) / 2
    
    if (y_ball and y_goalie):
        if (y_ball - y_goalie > 1):
            D = 250
            dir = 1
            arduino.write(bytes("1", 'utf-8'))
        elif (y_ball - y_goalie < -1):
            D = 250
            dir = 2
            arduino.write(bytes("2", 'utf-8'))
        else:
            D = 0
            dir = 0
            arduino.write(bytes("0", 'utf-8'))

    # if not contours_r and not contours_g: 
    #     arduino.write(bytes("2", 'utf-8'))
    #cv2.imshow("Mask", red_mask)
    cv2.imshow("Webcam", imageFrame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        arduino.close()
        break

cap.release()
cv2.destroyAllWindows()