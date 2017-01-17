"""
Created on Sep 27, 2015

@author: weston
"""

import numpy as np
import matplotlib.pyplot as plot

from motionless import LatLonMarker, DecoratedMap
import urllib
import cv2


class StaticMap(object):
    """
    Show the robot's location and sensor data, blocked versus unblocked areas.
    """
    
    def __init__(self):
        """What would be good initialization parameters to pass here?
        """
        self.__sizeX = 640
        self.__sizeY = 480
        # This scale may need to change if all the points don't fit in the 
        # image.
        self.__zoom = 18 
        self.__mapType = 'hybrid'
        self.__show = False
        plot.ion()
        return
    
    
    def getMapImage(self, currentLocation, missionLocations, mapName='location'):
        """Use the static maps API, add the waypoints to the map as markers,
        and return the image.
        """
        if currentLocation[0] != 0 or currentLocation[1] != 0:
            print 'Current location is initialized:'
            print 'Lat:', currentLocation[0], 'Lon:', currentLocation[1]
            coor1 = round(currentLocation[0], 4)
            coor2 = round(currentLocation[1], 4)
            
            dmap = DecoratedMap(lat=coor1,lon=coor2
                , size_x=self.__sizeX, size_y=self.__sizeY
                , zoom=self.__zoom, maptype=self.__mapType)
            
            marker = LatLonMarker(lat=coor1, lon=coor2, color='green', label='P')
            print 'Drawing waypoint position (P):', (coor1, coor2), '(green).'
            dmap.add_marker(marker)
        else:
            print 'Current location is uninitialized:'
            location = missionLocations[0]
            coor1 = round(location[0], 4)
            coor2 = round(location[1], 4)
            dmap = DecoratedMap(lat=coor1,lon=coor2
                , size_x=self.__sizeX, size_y=self.__sizeY
                , zoom=self.__zoom, maptype=self.__mapType)
        index = 1
        for location in missionLocations:
            coor1 = round(location[0], 4)
            coor2 = round(location[1], 4)
            print 'Lat:', coor1, 'Lon:', coor2
            if location[2] == 'waypoint':
                print 'Drawing waypoint marker:', index, '(blue).'
                color = 'blue'
            elif location[2] == 'target':
                print 'Drawing target marker:', index, '(red).'
                color = 'red'
            else:
                print 'Drawing other marker:', index, '(green).'
                color = 'green'
            label = str(index)
            index += 1
            marker = LatLonMarker(lat=coor1, lon=coor2, color=color, label=label)
            dmap.add_marker(marker)
        
        imageFileName = mapName + ".png"
        url = dmap.generate_url()
        print url
        urllib.urlretrieve(url, imageFileName)
        # Load an color image in grayscale
        img = cv2.imread(imageFileName, cv2.IMREAD_COLOR)
        
        if not self.__show:
            plot.clf()
            # cmap = 'gray', # TODO: figure out what color map to use.
            plot.imshow(img, interpolation = 'bicubic')
            plot.show()
            plot.draw()
        #cv2.destroyAllWindows()
        return img
        
        
        