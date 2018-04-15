"""
Created on Oct 20, 2015

@author: weston

DataVisualizer - Parent class for various sensor visualizers

InertialVisualizer - heading (mag + gyro), accelerometer, etc
SpacialVisualizer - sensor distances, encoder counts, speed, stuff
SystemVisualizer - GPS, vector, intertial and composit position, waypoints, map
"""

import numpy as np
#import matplotlib.pyplot as plt
import cv2, math

WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)

DEG_PER_RAD = 180.0 / math.pi

class DataVisualizer(object):
    """ Base class providing basic drawing functions. Inheritors can implement
    specialized graphical representations of their sensor data. 
    """
    FONT = cv2.FONT_HERSHEY_PLAIN
    
    def __init__(self, width, height, frameName):
        """
        """
        self.__width = width
        self.__height = height
        # Create a black image
        self.__img = np.zeros((width, height, 3), np.uint8)

        self.__frameName = frameName
        cv2.startWindowThread()
        cv2.namedWindow(self.__frameName, cv2.WINDOW_NORMAL)
        # Add trackbars to show different combinations of data.
        cv2.imshow(self.__frameName, self.__img)
        return
    
    
    def _drawCircle(self, x, y, radius, color, thickness):
        cv2.circle(self.__img, (x, y), radius, color, thickness)
        return
    
    
    def _drawLine(self, p1, p2, color, thickness):
        cv2.line(self.__img, p1, p2, color, thickness)
        return
    
    
    def _drawText(self, text, location, scale, color, thickness):
        cv2.putText(self.__img, text, location, self.FONT, scale, color, thickness)
        return
    
    
    def _clearFrame(self):
        self.__img = np.zeros((self.__height, self.__width, 3), np.uint8)
        return
    
    
    def _drawFrame(self):
        cv2.imshow(self.__frameName, self.__img)
    
    @property
    def width(self):
        return self.__width
    
    @property
    def height(self):
        return self.__height
    
    # TODO: provide functions to update the line locations for:
    # Compass Heading (calculated), XY raw: 1 window, 2 views
    # X Is lateral,Y is longitudinal:
    # Accelerometer vectors: XY (yaw), XZ (roll), YZ(pitch): 3 windows
    # Rate Gyroscope Heading, XY degrees per second. 2 views.
    
    # Estimated position based on integrating acceleration (twice)
    # acceleration -> Speed -> Distance, at each integration step create a
    # vector using the heading from the compass/gyro combination and sum it.
    # 


class InertialVisualizer(DataVisualizer):
    def __init__(self):
        """Constructor."""
        height = 500
        width = 500
        self.__compassRadius = 200
        self.__pitchRollRadius = 50
        DataVisualizer.__init__(self, width, height, 'Inertial Visualizer')
        
        self.update(0, 0, 0)
        return
    
    
    def update(self, heading, pitch, roll):
        """Draw the heading on the compass."""
        self._clearFrame()
        self.__drawBackground()
        self.__drawHeading(heading)
        self.__drawPitch(pitch)
        self.__drawRoll(roll)
        self._drawFrame()
        return
    
    
    def __drawHeading(self, heading):
        # Calculate the line from the origin to the perimiter of the circle.
        angle = (heading + 90) / DEG_PER_RAD
        xOffset = self.width / 2
        yOffset = self.height / 2
        radius = self.__compassRadius
        x = int(math.sin(angle) * radius + xOffset)
        y = int(math.cos(angle) * radius + yOffset)
        p1 = (xOffset, yOffset)
        p2 = (x, y)
        self._drawLine(p1, p2, BLUE, 4)
        self._drawCircle(x, y, 7, BLUE, -1)
        # Print the heading:
        loc = (25, 50)
        self._drawText(str(round(heading, 2)), loc, 3, WHITE, 1)
        return
    
    
    def __drawPitch(self, pitch):
        angle = (pitch + 90) / DEG_PER_RAD
        xOffset = self.width / 7
        yOffset = (self.height / 6) * 5
        self.__drawDoubleAngle(pitch, angle, xOffset, yOffset, 'P')
        return
    
    
    def __drawRoll(self, roll):
        angle = (roll + 90) / DEG_PER_RAD
        xOffset = (self.width / 7) * 6
        yOffset = (self.height / 6) * 5
        self.__drawDoubleAngle(roll, angle, xOffset, yOffset, 'R')
        return
    
    
    def __drawDoubleAngle(self, deg, rad, xOffset, yOffset, label):
        radius = self.__pitchRollRadius
        x = int(math.sin(rad) * radius + xOffset)
        y = int(math.cos(rad) * radius + yOffset)
        p1 = (xOffset, yOffset)
        p2 = (x, y)
        self._drawLine(p1, p2, BLUE, 4)
        x = int(math.sin(rad - math.pi) * radius + xOffset)
        y = int(math.cos(rad - math.pi) * radius + yOffset)
        p1 = (xOffset, yOffset)
        p2 = (x, y)
        self._drawLine(p1, p2, BLUE, 4)
        loc = (xOffset - 60, yOffset + 70)
        label = label + ':' + str(round(deg, 2))
        self._drawText(label, loc, 2, WHITE, 1)
        return
    
    
    def __drawBackground(self):
        """"""
        width = self.width
        height = self.height
        radius = self.__compassRadius
        self._drawCircle(height / 2, width / 2, radius, (100,100,100), -1)
        p1 = (0, height / 2)
        p2 = (width, height / 2)
        self._drawLine(p1, p2, GREEN, 2)
        p1 = (width / 2, 0)
        p2 = (width / 2, height)
        self._drawLine(p1, p2, GREEN, 2)
        self._drawText('N', (width / 2 - 20, 50), 4, WHITE, 2)
        self._drawText('E', (width - 50, height / 2 + 20), 4, WHITE, 2)
        self._drawText('S', (width / 2 - 20, height - 5), 4, WHITE, 2)
        self._drawText('W', (5, height / 2 + 20), 4, WHITE, 2)
        return
    
    
class SpacialVisualizer(DataVisualizer):
    """Draw the robot, draw the sensor ranges as lines
    """

    def __init__(self):
        """Constructor."""
        DataVisualizer.__init__(self, 500, 500, 'Spacial Visualizer')
        return


class SystemVisualizer(DataVisualizer):
    """Draw the robot, draw the sensor ranges as lines
    """

    def __init__(self):
        """Constructor."""
        DataVisualizer.__init__(self, 500, 500, 'System Visualizer')
        return


