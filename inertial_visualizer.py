'''
Created on Oct 20, 2015

@author: weston
'''

import numpy as np
import cv2


class InertialVisualizer(object):
    
    def __init__(self):
        # Create a black image
        self.__img = np.zeros((512, 512, 3), np.uint8)
        # Draw a diagonal blue line with thickness of 5 px
        cv2.line(self.__img,(0, 0), (511, 511), (255, 0, 0), 5)
        
        cv2.startWindowThread()
        self.__frameName = 'Inertial Visualizer'
        cv2.namedWindow(self.__frameName, cv2.WINDOW_NORMAL)
        # Add trackbars to show different combinations of data.
        cv2.imshow(self.__frameName, self.__img)
        pass

    # TODO: provide functions to update the line locations for:
    # Compass Heading (calculated), XY raw: 1 window, 2 views
    # X Is lateral,Y is longitudinal:
    # Accelerometer vectors: XY (yaw), XZ (roll), YZ(pitch): 3 windows
    # Rate Gyroscope Heading, XY degrees per second. 2 views.
    
    # Estimated position based on integrating acceleration (twice)
    # acceleration -> Speed -> Distance, at each integration step create a
    # vector using the heading from the compass/gyro combination and sum it.
    # 
    #  
    
    
foo = InertialVisualizer()