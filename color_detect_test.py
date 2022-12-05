import cv2
import numpy as np
from matplotlib import pyplot as plt
import serial 
import time

def color_detect(): 
    print("Opening camera, please wait ~ 1 minute")
    cap = cv2.VideoCapture(0)
    print(f"Camera ready: {cap.isOpened()}")

    counter = 0
    last = time.time()

    while True: 

        while cap.isOpened():
            counter += 1
            _, imageFrame = cap.read()
            hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
            
            blue_lower = np.array([90, 50, 70], np.uint8)
            blue_upper = np.array([115, 255, 255], np.uint8)
            blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
            
            # red_lower_0 = np.array([0, 100, 100], np.uint8)
            # red_upper_0 = np.array([10, 255, 255], np.uint8)
            # red_mask_0 = cv2.inRange(hsvFrame, red_lower_0, red_upper_0)
            
            # red_lower_1 = np.array([160, 100, 100], np.uint8)
            # red_upper_1 = np.array([179, 255, 255], np.uint8)
            # red_mask_1 = cv2.inRange(hsvFrame, red_lower_1, red_upper_1)
            
            # red_mask = red_mask_0 + red_mask_1
            
            green_lower = np.array([60, 100, 50], np.uint8)
            green_upper = np.array([80, 255, 255], np.uint8)
            green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

            kernal = np.ones((5, 5), "uint8")
            
            # red_mask = cv2.dilate(red_mask, kernal)
            # res_red = cv2.bitwise_and(imageFrame, imageFrame, mask = red_mask)
            
            blue_mask = cv2.dilate(blue_mask, kernal)
            res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask = blue_mask)
            
            green_mask = cv2.dilate(green_mask, kernal)
            res_green = cv2.bitwise_and(imageFrame, imageFrame, mask = green_mask)
            
            # Creating contour to track  color
            contours_b, hierarchy_b = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_g, hierarchy_g = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # contours_r, hierarchy_r = cv2.findContours(red_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            
            # if contours_r:
            #     red_max_contour = max(contours_r, key = cv2.contourArea)
            #     x_r_max, y_r_max, w_r_max, h_r_max = cv2.boundingRect(red_max_contour)
            #     imageFrame = cv2.rectangle(imageFrame, (x_r_max, y_r_max), (x_r_max + w_r_max, y_r_max + h_r_max), (0, 0, 255), 2)
            #     cv2.putText(imageFrame, f"Center Coordinate: {(x_r_max + w_r_max) / 2}, {(y_r_max + h_r_max) / 2}", (x_r_max, y_r_max), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0, 255))
                
            if contours_g:
                green_max_contour = max(contours_g, key = cv2.contourArea)
                x_g_max, y_g_max, w_g_max, h_g_max = cv2.boundingRect(green_max_contour)
                imageFrame = cv2.rectangle(imageFrame, (x_g_max, y_g_max), (x_g_max + w_g_max, y_g_max + h_g_max), (0, 255, 0), 2)
                cv2.putText(imageFrame, f"Center Coordinate: {(x_g_max + w_g_max) / 2}, {(y_g_max + h_g_max) / 2}", (x_g_max, y_g_max), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0))
                cv2.putText(imageFrame, f"Big green boy: x = {x_g_max}, y = {y_g_max}",(50, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))

            if contours_b:
                blue_max_contour = max(contours_b, key = cv2.contourArea)
                x_b_max, y_b_max, w_b_max, h_b_max = cv2.boundingRect(blue_max_contour)
                imageFrame = cv2.rectangle(imageFrame, (x_b_max, y_b_max), (x_b_max + w_b_max, y_b_max + h_b_max), (255, 0, 0), 2)
                cv2.putText(imageFrame, f"Center Coordinate: {(x_b_max + w_b_max) / 2}, {(y_b_max + h_b_max) / 2}", (x_b_max, y_b_max), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0))
                cv2.putText(imageFrame, f"Big blue boy: {x_b_max} {y_b_max}",(50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))
                
            # if contours_g and contours_b:
            #     dist_red2green = np.sqrt((x_b_max - x_g_max)**2 + (y_b_max - y_g_max)**2 )
            #     cv2.putText(imageFrame, f"Big blue boy: {x_b_max} {y_b_max}",(50, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))
            #     cv2.putText(imageFrame, f"Big green boy: x = {x_g_max}, y = {y_g_max}",(50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))
            #     #v2.putText(imageFrame, f"Distance: {dist_red2green}", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))

            cv2.imshow("Webcam", imageFrame)

            if time.time() - last > 10:
                print(counter)
                last = time.time()
                counter = 0
                        
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

color_detect()