import cv2
import numpy as np
from matplotlib import pyplot as plt
import serial 
import time
import math


def regress(x, y):
    x = np.array(x)
    y = np.array(y)

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


print("Opening camera, please wait ~ 1 minute")
cap = cv2.VideoCapture(0)
print(f"Camera ready: {cap.isOpened()}")

#x_ball, y_ball, x_goalie, y_goalie = 0, 0, 0, 0

start = time.time()

HIGH_LIMIT = 149
LOW_LIMIT = 68

x_ball = [0]
y_ball = [(HIGH_LIMIT-LOW_LIMIT)/2]

x_goalie = [0]
y_goalie = [(HIGH_LIMIT-LOW_LIMIT)/2]

time_vec = [time.time()]

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
            # x_ball = (x_g_max + w_g_max) / 2
            # y_ball = (y_g_max + h_g_max) / 2
            
            #D = 250
            #arduino.write(bytes("250", 'utf-8'))
            #time.sleep(0.05)
            #datag = arduino.readline()
            #print(datag)
            x_ball.append((x_g_max + w_g_max) / 2)
            y_ball.append((y_g_max + h_g_max) / 2)
        else:
            x_ball.append(0)
            y_ball.append(0)
    
    if contours_b:
        blue_max_contour = max(contours_b, key = cv2.contourArea)
        x_b_max, y_b_max, w_b_max, h_b_max = cv2.boundingRect(blue_max_contour)
        imageFrame = cv2.rectangle(imageFrame, (x_b_max, y_b_max), (x_b_max + w_b_max, y_b_max + h_b_max), (255, 0, 0), 2)
        cv2.putText(imageFrame, f"Center Coordinate: {(x_b_max + w_b_max) / 2}, {(y_b_max + h_b_max) / 2}", (x_b_max, y_b_max), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0))
        cv2.putText(imageFrame, f"Big blue boy: {x_b_max} {y_b_max}",(50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))
        # x_goalie = (x_b_max + w_b_max) / 2
        # y_goalie = (y_b_max + h_b_max) / 2
        x_goalie.append((x_b_max + w_b_max) / 2)
        y_goalie.append((y_b_max + h_b_max) / 2)
    else:
        x_goalie.append(0)
        y_goalie.append(0)  

    time_vec.append(time.time()-start)




    
    # if (y_ball and y_goalie):
    #     if (y_ball - y_goalie > 1) and (y_goalie < HIGH_LIMIT):
    #         arduino.write(bytes("1", 'utf-8'))
    #         print('go negative')
    #     elif (y_ball - y_goalie < -1) and (y_goalie > LOW_LIMIT):
    #         arduino.write(bytes("2", 'utf-8'))
    #         print('go positive')
    #     else:
    #         arduino.write(bytes("0", 'utf-8'))
    #         print('dont move oyu bitch')

    # if not contours_r and not contours_g: 
    #     arduino.write(bytes("2", 'utf-8'))
    #cv2.imshow("Mask", red_mask)
    cv2.imshow("Webcam", imageFrame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        arduino.close()
        break


    m_xy, b_xy = regress(x_ball[-10:], y_ball[-10:])

    # print(time_vec[-10:])
    # print(x_ball[-10:])

    m_xt, b_xt = regress(time_vec[-10:], x_ball[-10:])

    currGoalieY = y_goalie[-1]
    desiredGoalieY = m_xy*x_goalie[-1] + b_xy

    if math.isnan(desiredGoalieY) or m_xt > 0:    # maybe add      or (abs(m_xy) > 5) to not freak out when its bouncing a lot
        desiredGoalieY = y_ball[-1]
                
    desiredGoalieY = max(min(desiredGoalieY, HIGH_LIMIT), LOW_LIMIT)

    if currGoalieY - desiredGoalieY > 1:
        arduino.write(bytes("2", 'utf-8'))
    elif currGoalieY - desiredGoalieY < -1:
        arduino.write(bytes("1", 'utf-8'))
    else:
        arduino.write(bytes("0", 'utf-8'))



cap.release()
cv2.destroyAllWindows()
