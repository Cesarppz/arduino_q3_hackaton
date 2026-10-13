import numpy as np
import cv2

def get_color_limits(color):
    c = np.uint8([[color]]) #here insert the bgr   values of the color you want to covert to hsv
    hsv_color = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    lowerlimit = hsv_color[0][0][0] - 10, 110, 100
    upperlimit = hsv_color[0][0][0] + 10, 255, 255

    lower_limit = np.array(lowerlimit, dtype=np.uint8)
    upper_limit = np.array(upperlimit, dtype=np.uint8)

    return lower_limit, upper_limit