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
    pos_vals = np.zeros(5)


    try:
        while True:
            pos_vals = np.vstack(color_detect(), pos_vals)  #call the vision code, add current ball x,
                                                            #ball y, goalie x, goalie y, and time.
            ballx = pos_vals[0:4, 0]
            bally = pos_vals[0:4, 1]
            time = pos_vals[0:4, 4]

            (x_coeff1, x_coeff0) = regress(time, ballx)


            currGoalie = pos_vals[0][4]
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



