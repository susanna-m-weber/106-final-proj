import cv2
import numpy as np
from matplotlib import pyplot as plt
import serial
import time
import serial
from color_detect_test import *

if __name__ == "__main__":
    main()

arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1) # change port as needed


def main():
    print("Opening camera, please wait ~ 1 minute")
    cap = cv2.VideoCapture(0)
    print(f"Camera ready: {cap.isOpened()}")

    start = time.time()

    x_ball = []
    y_ball = []

    x_goalie = []
    y_goalie = []

    time_vec = []


    try:
        while cap.isOpened():
            _, imageFrame = cap.read()
            hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

            blue_lower = np.array([90, 50, 70], np.uint8)
            blue_upper = np.array([115, 255, 255], np.uint8)
            blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

            green_lower = np.array([60, 100, 50], np.uint8)
            green_upper = np.array([80, 255, 255], np.uint8)
            green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

            kernal = np.ones((5, 5), "uint8")

            blue_mask = cv2.dilate(blue_mask, kernal)
            res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask=blue_mask)

            green_mask = cv2.dilate(green_mask, kernal)
            res_green = cv2.bitwise_and(imageFrame, imageFrame, mask=green_mask)

            contours_b, hierarchy_b = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_g, hierarchy_g = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours_g:
                green_max_contour = max(contours_g, key = cv2.contourArea)
                x_g_max, y_g_max, w_g_max, h_g_max = cv2.boundingRect(green_max_contour)
                imageFrame = cv2.rectangle(imageFrame, (x_g_max, y_g_max), (x_g_max + w_g_max, y_g_max + h_g_max), (0, 255, 0), 2)
                cv2.putText(imageFrame, f"Center Coordinate: {(x_g_max + w_g_max) / 2}, {(y_g_max + h_g_max) / 2}", (x_g_max, y_g_max), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0))
                cv2.putText(imageFrame, f"Big green boy: x = {x_g_max}, y = {y_g_max}",(50, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))
            x_ball.append((x_g_max + w_g_max) / 2)
            y_ball.append((y_g_max + h_g_max) / 2)

            if contours_b:
                blue_max_contour = max(contours_b, key = cv2.contourArea)
                x_b_max, y_b_max, w_b_max, h_b_max = cv2.boundingRect(blue_max_contour)
                imageFrame = cv2.rectangle(imageFrame, (x_b_max, y_b_max), (x_b_max + w_b_max, y_b_max + h_b_max), (255, 0, 0), 2)
                cv2.putText(imageFrame, f"Center Coordinate: {(x_b_max + w_b_max) / 2}, {(y_b_max + h_b_max) / 2}", (x_b_max, y_b_max), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0))
                cv2.putText(imageFrame, f"Big blue boy: {x_b_max} {y_b_max}",(50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))
            x_goalie.append((x_b_max + w_b_max) / 2)
            y_goalie.append((y_b_max + h_b_max) / 2)

            time_vec.append(time.time()-start)


            cv2.imshow("Webcam", imageFrame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break



            currGoalieY =
            desiredGoalie = 0
            
            goalie_data = bytearray([currGoalie, desiredGoalie])
            arduino.write(goalie_data)
            time.sleep(0.05)


    except KeyboardInterrupt:
        pass


def regress(x, y):
    # number of observations/points
    n = np.size(x)

    # mean of x and y vector
    m_x = np.mean(x)
    m_y = np.mean(y)

    # calculating cross-deviation and deviation about x
    SS_xy = np.sum(y * x) - n * m_y * m_x
    SS_xx = np.sum(x * x) - n * m_x * m_x

    # calculating regression coefficients
    b_1 = SS_xy / SS_xx
    b_0 = m_y - b_1 * m_x

    return (b_0, b_1)



