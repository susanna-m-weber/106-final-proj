import cv2
import numpy as np
from matplotlib import pyplot as plt
import serial 
import time

print("Opening camera, please wait ~ 1 minute")
cap = cv2.VideoCapture(0)
print(f"Camera ready: {cap.isOpened()}")

arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)

while cap.isOpened():
    _, imageFrame = cap.read()
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    
    blue_lower = np.array([90, 50, 70], np.uint8)
    blue_upper = np.array([115, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
    
    red_lower_0 = np.array([0, 100, 100], np.uint8)
    red_upper_0 = np.array([10, 255, 255], np.uint8)
    red_mask_0 = cv2.inRange(hsvFrame, red_lower_0, red_upper_0)
    
    red_lower_1 = np.array([160, 100, 100], np.uint8)
    red_upper_1 = np.array([179, 255, 255], np.uint8)
    red_mask_1 = cv2.inRange(hsvFrame, red_lower_1, red_upper_1)
    
    red_mask = red_mask_0 + red_mask_1
    
    green_lower = np.array([60, 100, 50], np.uint8)
    green_upper = np.array([80, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    kernal = np.ones((5, 5), "uint8")
    
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, mask = red_mask)
    
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask = blue_mask)
    
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame, mask = green_mask)
    
    # Creating contour to track  color
    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_g, hierarchy_g = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_r, hierarchy_r = cv2.findContours(red_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    if contours_r:
        red_max_contour = max(contours_r, key = cv2.contourArea)
        x_r_max, y_r_max, w_r_max, h_r_max = cv2.boundingRect(red_max_contour)
        imageFrame = cv2.rectangle(imageFrame, (x_r_max, y_r_max), (x_r_max + w_r_max, y_r_max + h_r_max), (0, 0, 255), 2)
        cv2.putText(imageFrame, f"Center Coordinate: {(x_r_max + w_r_max) / 2}, {(y_r_max + h_r_max) / 2}", (x_r_max, y_r_max), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0, 255))
        
        arduino.write(bytes("1", 'utf-8'))
        time.sleep(0.05)
        datar = arduino.readline()
        print(datar)
        
    if contours_g:
        green_max_contour = max(contours_g, key = cv2.contourArea)
        x_g_max, y_g_max, w_g_max, h_g_max = cv2.boundingRect(green_max_contour)
        imageFrame = cv2.rectangle(imageFrame, (x_g_max, y_g_max), (x_g_max + w_g_max, y_g_max + h_g_max), (0, 255, 0), 2)
        cv2.putText(imageFrame, f"Center Coordinate: {(x_g_max + w_g_max) / 2}, {(y_g_max + h_g_max) / 2}", (x_g_max, y_g_max), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0))
        
        arduino.write(bytes("0", 'utf-8'))
        time.sleep(0.05)
        datag = arduino.readline()
        print(datag)
    
    # if not contours_r and not contours_g: 
    #     arduino.write(bytes("2", 'utf-8'))
        
    if contours_r and contours_g:
        dist_red2green = np.sqrt((x_r_max - x_g_max)**2 + (y_r_max - y_g_max)**2 )
        cv2.putText(imageFrame, f"Big red boy: {x_r_max}, {y_r_max}",(50, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))
        cv2.putText(imageFrame, f"Big green boy: {x_g_max}, {y_g_max}",(50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))
        cv2.putText(imageFrame, f"Distance: {dist_red2green}", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))
    
    #cv2.imshow("Mask", red_mask)
    cv2.imshow("Webcam", imageFrame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        arduino.close()
        break

cap.release()
cv2.destroyAllWindows()