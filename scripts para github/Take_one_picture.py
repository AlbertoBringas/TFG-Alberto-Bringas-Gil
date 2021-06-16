##: ESTE SCRIPT SIRVE PARA SACAR UNA SOLA IMAGEN DE LA CÁMARA DE AIRSIM. PUEDE SER ÚTIL PARA OBTENER LAS IMÁGENES NECESARIAS PARA LA CALIBRACIÓN
##: ---------------------------------------------------------------
##: THIS SCRIPT TAKES ONE PICTURE FROM THE AIRSIM CAMERA, IT MAY BE HELPFUL TO OBTAIN ALL IMAGES FOR CALIBRATION PROCESS


import airsim
import matplotlib.pyplot as plt
import matplotlib as mpl
from cv2 import aruco
import numpy as np
import os
# requires Python 3.5.3 :: Anaconda 4.4.0
# pip install opencv-python
import cv2
import time
import sys
import math


client = airsim.MultirotorClient()
client.confirmConnection()






responses = client.simGetImages([airsim.ImageRequest("front_center_custom", airsim.ImageType.Scene, False, False)])
response = responses[0]

# get numpy array
img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 

# reshape array to 4 channel image array H X W X 4
img_rgb = img1d.reshape(response.height, response.width, 3)

# original image is fliped vertically
# img_rgb = np.flipud(img_rgb)


# write to png 
airsim.write_png(os.path.normpath("Documents28" + '.png'), img_rgb)




