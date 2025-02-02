import cv2
from PIL import Image as Im
import matplotlib.pyplot as plt
import numpy as np
import rospy
import random as rng


def getMask(img):
    #get mask specific for dark green trashcans

    rng.seed(12345)
    img = np.array(img)
    
    lower = np.array([0, 0, 0])
    upper = np.array([255, 255, 255])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(img, img, mask=mask)
    filtered = cv2.bitwise_and(img, img, mask=mask)

    grayImage = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
    (thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 0, 128, cv2.THRESH_BINARY)
    #Im.fromarray(blackAndWhiteImage).show()

    return blackAndWhiteImage


    
def num_pixels(im):
    
    #find pixels
    number_of_white_pix = np.sum(im == 255)
    number_of_black_pix = np.sum(im == 0)
    num_pix = np.sum(im)
    
    return number_of_black_pix, num_pix

    
    
