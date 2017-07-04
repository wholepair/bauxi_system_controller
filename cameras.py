"""
Created on Sep 27, 2015

@author: weston
"""


# import the necessary packages
import cv2
import numpy as np

import os

if os.uname()[4][:3] == 'arm':
    from picamera.array import PiRGBArray
    from picamera import PiCamera

import time

def getCvVersion():
    return cv2.__version__


###############################################################################
# CAMERA: generic base class.
###############################################################################

class Camera(object):
    
    MIN_AREA_THRESHOLD = 100
    FONT = cv2.FONT_HERSHEY_PLAIN
    
    def __init__(self, device, filterProperties):
        """ All cameras share a capture device and the ability to query it's
        state (open/closed)
        """
        self._deviceName = str(device)
        self._frameName = self._deviceName + ' Frame'
        self._maskName = self._deviceName + ' Mask'
        self._resName = self._deviceName + ' Res'
        
        self.__updatingTrackbar = False
        self._cap = None
        
        self._hue = filterProperties[0]
        self._sat = filterProperties[1]
        self._brt = filterProperties[2]
        
        self._threshHue = filterProperties[3]
        self._threshSat = filterProperties[4]
        self._threshBrt = filterProperties[5]
        
        self.__updateColorThresholds()
        
        self.__showWindow = False
        self.__windowsInit = False
        self.__returnImage = False
        self.__imageData = None
        
        # Set flag to save images on next frame, then it goes back to false.
        self.__saveImages = False
        
        # Change state through mouse click event.
        self.__getColor = False
        
        self.__targetColorVisible = False
        self.__targetColorX = -1
        self.__targetColorY = -1
        
        self.__targetShapeVisible = False
        self.__targetShapeX = -1
        self.__targetShapeY = -1
        # Number of triangular targets.
        self.__targetCount = 0
        # Turn this off for no X
        #self.showImages(True)
        return
    
    
    def __clampColorComponent(self, component, threshold, name):
        clamped = True
        if component - threshold < 0:
            print name, 'minus threshold < 0, clamping value:', component,
            component = threshold
            print ',', name, 'is now', component, ', threshold', threshold
        elif component + threshold > 255:
            print name, 'plus threshold > 255, clamping value:', component,
            component = 255 - threshold
            print ',', name, 'is now', component, ', threshold', threshold
        else:
            clamped = False
        return component, clamped
    
    
    def __updateColorThresholds(self):
        """ Must have hue sat and brt fields initialized.
        """
        self._hue, ch = self.__clampColorComponent(self._hue
                , self._threshHue, "hue")
        
        self._sat, cs = self.__clampColorComponent(self._sat
                , self._threshSat, "saturation")
        
        self._brt, cv = self.__clampColorComponent(self._brt
                , self._threshBrt, "brightness")
        
        print 'New HSB:', self._hue, self._sat, self._brt
        print 'HSB Thresholds:', self._threshHue, self._threshSat, self._threshBrt
        
        # define range of color in HSV
        self._lowerColor = np.array([self._hue - self._threshHue
                                   , self._sat - self._threshSat
                                   , self._brt - self._threshBrt])
        
        self._upperColor = np.array([self._hue + self._threshHue
                                   , self._sat + self._threshSat
                                   , self._brt + self._threshBrt])
        
        print "Filter upper bounds:", self._upperColor
        print "Filter lower bounds:", self._lowerColor
        return ch or cs or cv # If we moved a component, we need to update UI.
    
    
    def _findTarget(self, frame):
        """ Locate the target color in the frame.
        """
        if self.__getColor == True:
            self.__getColor = False
            blue = int(frame[self.__colorY][self.__colorX][0])
            green = int(frame[self.__colorY][self.__colorX][1])
            red = int(frame[self.__colorY][self.__colorX][2])
            colorHsb = rgbToHsb(red, green, blue)[0][0]
            self._hue, self._sat, self._brt = colorHsb
            print 'X, Y:', self.__colorX, self.__colorY
            print 'New RGB:', red, green, blue
            print 'New HSB:', self._hue, self._sat, self._brt
            cv2.circle(frame,(self.__colorX, self.__colorY), 20, (blue,green,red), -1)
            if os.uname()[4][:3] != 'arm':
                # Setting the positions also updates HSV.
                self.__updatingTrackbar = True
                self.__updateColorThresholds()
                self.__setTrackbarPosColor()
                self.__updatingTrackbar = False
            else:
                self.__updateColorThresholds()
            
        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Threshold the HSV image to get only orange colors
        mask = cv2.inRange(hsvImage, self._lowerColor, self._upperColor)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Find the centroid of the masked image.
        M = cv2.moments(mask)
        if M['m00'] != 0:
            self.__targetColorVisible = True
            cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
            # Annotate the image with a circle in the centroid.
            cv2.circle(frame, (cx, cy), 10, (255, 0, 0), -1)
            # Center of mass of color:
            self.__targetColorX = cx
            self.__targetColorY = cy
        else:
            self.__targetColorVisible = False
            # Maybe also set the X and Y coords to -1 too.

        (_, contours, _) = cv2.findContours(mask.copy()
            , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Sort the contour list by area from smallest to largest so that
        # the coordinates of the last shape that matches will be kept.
        contours = sorted(contours, key = cv2.contourArea)
        
        self.__targetCount = 0
        self.__targetShapeVisible = False
        self.__targetScale = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.MIN_AREA_THRESHOLD: continue
            cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
            epsilon = 0.05 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            # Draw the approximate polygon. If it has three sides, it is a cone?
            if len(approx) == 3 or len(approx) == 4:
                self.__targetShapeVisible = True
                self.__targetScale = area
                cv2.polylines(frame, [approx], True, (0,255,255))
                self.__targetCount += 1
            
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                # Annotate the image with a circle in the centroid.
                if len(approx) == 3:
                    cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
                    cv2.putText(frame,'Cone',(cx,cy), self.FONT, 4,(255,255,255),2)
                    self.__targetShapeX = cx
                    self.__targetShapeY = cy
                else:
                    cv2.circle(frame, (cx, cy), 3, (255, 0, 0), -1)
            
        
        if self.__saveImages == True:
            self.__saveImages = False
            print 'Saving images: frame.jpg, mask.jpg, res.jpg.'
            timeStamp = str(time.time())
            cv2.imwrite('frame' + timeStamp + '.jpg', frame)
            cv2.imwrite('mask' + timeStamp + '.jpg', mask)
            cv2.imwrite('res' + timeStamp + '.jpg', res)
        
        if self.__showWindow == True:
            if self.__windowsInit == False:
                self.__setupImageWindows()
            # Show the frame after we annotate it.
            cv2.imshow(self._frameName, frame)
            cv2.imshow(self._maskName, mask)
            cv2.imshow(self._resName, res)
            #time.sleep(0.1)
            #cv2.waitKey(25) # This causes a crash if running in a thread.
        
        if self.__returnImage:
            self.__imageData = frame
            self.__returnImage = False
        else:
            self.__imageData = None
        
        # Return: color visible, colorX, colorY, shape visible, shapeX, shapeY, scale, count 
        targetInfo = (self.__targetColorVisible
                    , self.__targetColorX
                    , self.__targetColorY
                    , self.__targetShapeVisible
                    , self.__targetShapeX
                    , self.__targetShapeY
                    , self.__targetScale
                    , self.__targetCount
                    , self.__imageData)
        
        return targetInfo
    
    
    def __trackbarChanged(self, val):
        """Set the hue, saturation, and value and thresholds based on changing
        the trackpbar sliders
        """
        if self.__updatingTrackbar:
            return
        
        self._hue = cv2.getTrackbarPos('H', self._resName)
        self._sat = cv2.getTrackbarPos('S', self._resName)
        self._brt = cv2.getTrackbarPos('B', self._resName)
    
        self._threshHue = cv2.getTrackbarPos('HT', self._resName)
        self._threshSat = cv2.getTrackbarPos('ST', self._resName)
        self._threshBrt = cv2.getTrackbarPos('BT', self._resName)
        
        clamped = self.__updateColorThresholds()
        
        if clamped:
            self.__setTrackbarPosColor()
        
        return
    
    
    def __setTrackbarPosColor(self):
        """ Set the trackbar locations for HSV."""
        cv2.setTrackbarPos('H', self._resName, self._hue)
        cv2.setTrackbarPos('S', self._resName, self._sat)
        cv2.setTrackbarPos('B', self._resName, self._brt)
        return
    
    
    def __setTrackbarPosThresh(self):
        """ Set the trackbar locations for HSV thresholds."""
        cv2.setTrackbarPos('HT', self._resName, self._threshHue)
        cv2.setTrackbarPos('ST', self._resName, self._threshSat)
        cv2.setTrackbarPos('BT', self._resName, self._threshBrt)
        return
    
    
    # mouse callback function
    def __mouseCallback(self, event, x, y, flags, param):
        """ Use the mouse to select colors."""
        if event != cv2.EVENT_MOUSEMOVE:
            # Store the image coordinates. 
            self.__colorX = x
            self.__colorY = y
            self.__getColor = True
            return
    
    
    def showImages(self, show=True):
        """"""
        self.__showWindow = show
        return
    
    
    def __setupImageWindows(self):
        print 'Setting up image window.'
        if self.__windowsInit == False:
            print 'Initializing window controls.'
            # Conditionally start the window thread.
            cv2.startWindowThread()
            
            # Conditionally show windows.
            cv2.namedWindow(self._frameName, cv2.WINDOW_NORMAL)
            cv2.namedWindow(self._maskName, cv2.WINDOW_NORMAL)
            cv2.namedWindow(self._resName, cv2.WINDOW_NORMAL)
            self.__windowsInit = True
            if os.uname()[4][:3] != 'arm':
                # Filter color
                cv2.createTrackbar('H', self._resName, 0, 255, self.__trackbarChanged)
                cv2.createTrackbar('S', self._resName, 0, 255, self.__trackbarChanged)
                cv2.createTrackbar('B', self._resName, 0, 255, self.__trackbarChanged)
                # Filter color
                cv2.createTrackbar('HT', self._resName, 0, 127, self.__trackbarChanged)
                cv2.createTrackbar('ST', self._resName, 0, 127, self.__trackbarChanged)
                cv2.createTrackbar('BT', self._resName, 0, 127, self.__trackbarChanged)
                
                self.__updatingTrackbar = True
                self.__setTrackbarPosColor()
                self.__setTrackbarPosThresh()
                self.__updatingTrackbar = False
            
            cv2.setMouseCallback(self._frameName, self.__mouseCallback)
        else:
            print 'Window is already initialized.'
        return
    
    
    def saveImages(self):
        # Save the images the next time a picture is taken.
        self.__saveImages = True
        return
    
    
    def returnImage(self):
        # Return the next frame as a field within the camera message.
        self.__returnImage = True
        return
    
    
    @property
    def targetColorVisible(self):
        return self.__targetColorVisible
    
    @property
    def targetShapeVisible(self):
        return self.__targetShapeVisible
    
    @property
    def targetColorX(self):
        return self.__targetColorX

    @property
    def targetColorY(self):
        return self.__targetColorY
    
    @property
    def targetShapeX(self):
        return self.__targetShapeX

    @property
    def targetShapeY(self):
        return self.__targetShapeY
    
    @property
    def targetCount(self):
        """ The number of potential targets we are tracking.
        """
        return self.__targetCount
    
    @property
    def targetScale(self):
        return self.__targetScale
    
    @property
    def showWindow(self):
        return self.__showWindow
    

###############################################################################
# RASPBERRY PI CAMERA:
###############################################################################

class RaspPiCamera(Camera):
    
    def __init__(self, device, filterProperties):
        """ Constructor.
        """
        Camera.__init__(self, device, filterProperties)
        # initialize the camera and grab a reference to the raw camera capture
        self._cap = PiCamera()
        self._cap.resolution = (640, 480)
        self._cap.framerate = 32
        self.__rawCapture = PiRGBArray(self._cap, size=self._cap.resolution)
        
        #self._cap.start_preview()
        # allow the camera to warmup
        time.sleep(2)
        return
    
    
    def isOpen(self):
        """ Camera open flag.
        """
        return not self._cap.closed
    
    
    def takePicture(self):
        # grab an image from the camera
        self._cap.capture(self.__rawCapture, format='bgr')
        frame = self.__rawCapture.array
        
        # Returns result frame read.
        targetInfo = self._findTarget(frame)
        self.__rawCapture.truncate(0)
        return targetInfo
    
    
    def close(self):
        """ Close the camera device.
        """
        self._cap.close()
        return
    

###############################################################################
# USB CAMERA (WEBCAM):
###############################################################################

class WebCamera(Camera):
    
    def __init__(self, device, filterProperties):
        """ Constructor
        """
        Camera.__init__(self, device, filterProperties)
        self._cap = cv2.VideoCapture(device)
        # http://stackoverflow.com/questions/11420748/setting-camera-parameters-in-opencv-python
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        time.sleep(0.1)
        if not self._cap.isOpened():
            print "Cannot open camera!"
        return
    
    
    def isOpen(self):
        """ Camera open flag.
        """
        return self._cap.isOpened()
    
    
    def takePicture(self):
        """ Take a picture, return target tracking data.
        """
        ret, frame = self._cap.read()
        # Returns result frame read.
        if ret == True:
            targetInfo = self._findTarget(frame)
        else:
            targetInfo = None
        return targetInfo
    
    
    def close(self):
        """Release and close the camera device.
        """
        print 'Releasing:', repr(self)
        self._cap.release()
        return
    

###############################################################################
# CAMERA WRAPPER: give the camera a serial interface.
###############################################################################

# TODO: implement wrapper class to turn the camera's into serial ports :)
# implement readline, open close etc..
class CameraWrapper(object):
    
    ID_CAMERA = 'c'
    
    def __init__(self, device, filterProperties):
        self.__device = None
        if device == 'PiCamera':
            self.__device = RaspPiCamera(device, filterProperties)
        else:
            self.__device = WebCamera(int(device[-1]), filterProperties)
        return
    
    
    def isOpen(self):
        return self.__device.isOpen()
    
    
    def readline(self):
        # Used to acquire an image from the camera.
        return (self.ID_CAMERA, self.__device.takePicture())
    
    
    def write(self, data):
        # Used to show the image viewer windows.
        if data == 'show':
            print 'Camera Wrapper: Got show images command.'
            self.__device.showImages(True)
        elif data == 'pic':
            print 'Camera Wrapper: Got picture taking command.'
            self.__device.takePicture()
        elif data == 'save':
            print 'Camera Wrapper: Got save images command.'
            # Single shot, save an image.
            self.__device.saveImages()
        elif data == 'return':
            print 'Camera Wrapper: Got return image data command.'
            # The next camera message will contain image data.
            self.__device.returnImage()
        return
    
    
    def takePicture(self):
        """Primary API call, image capture command"""
        # Used to acquire an image from the camera.
        return self.__device.takePicture()
    
    
    def showImages(self, show=True):
        """Set to true to setup image viewer windows and show images."""
        self.__device.showImages(show)
        return
    
    
    def saveImages(self, show=True):
        """Save the next set of acquired images."""
        self.__device.saveImages()
        return
    
    
    def returnImage(self):
        """Return the next image as a field in the message."""
        self.__device.returnImage()
        return
    
    
    def close(self):
        print 'Camera Wrapper: closing camera device.'
        self.__device.close()
        print 'Camera Wrapper: closed camera.'
        return
    

###############################################################################
# HELPER FUNCTIONS:
###############################################################################

def rgbToHsb(red, green, blue):
    green = np.uint8([[[ blue, green, red ]]])
    return cv2.cvtColor(green,cv2.COLOR_BGR2HSV)


if __name__ == "__main__":
    # Instantiate the data processor as the parent.
    cam = CameraWrapper('/dev/video0', (22, 128, 127, 10, 50, 127))
    cam.showImages(True)
    cam.readline()
    #cam.close()


