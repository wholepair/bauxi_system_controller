"""
Created on Sep 27, 2015

@author: weston
"""


# import the necessary packages
import cv2
import numpy as np

isPyCamera = False
import os
if os.uname()[4][:3] == 'arm':
    from picamera.array import PiRGBArray
    from picamera import PiCamera
    isPyCamera = True
else:
    import cv # Used by x86 version?
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
        
        self._cap = None
        
        self._hue = filterProperties[0]
        self._sat = filterProperties[1]
        self._val = filterProperties[2]
        
        self._threshHue = filterProperties[3]
        self._threshSat = filterProperties[4]
        self._threshVal = filterProperties[5]
        
        self.__updateColorThresholds(self._hue, self._sat, self._val)
        
        self.__showWindow = False
        
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
        
        self.showImages(True)
        return
    
    def __updateColorThresholds(self, hue, sat, val):
        """ Must have hue sat and val fields initialized.
        """
        self._hue = hue
        self._sat = sat
        self._val = val
        # define range of color in HSV
        self._lowerColor = np.array([hue - self._threshHue
                                   , sat - self._threshSat
                                   , val - self._threshVal])
        
        self._upperColor = np.array([hue + self._threshHue
                                   , sat + self._threshSat
                                   , val + self._threshVal])
        return
    
    
    def _findTarget(self, frame):
        """ Locate the target color in the frame.
        """
        if self.__getColor == True:
            self.__getColor = False
            blue = int(frame[self.__colorY][self.__colorX][0])
            green = int(frame[self.__colorY][self.__colorX][1])
            red = int(frame[self.__colorY][self.__colorX][2])
            colorHsv = yuvToRgb(red, green, blue)[0][0]
            self._hue, self._sat, self._val = colorHsv
            print 'X, Y:', self.__colorX, self.__colorY
            print 'New RGB:', red, green, blue
            print 'New HSV:', self._hue, self._sat, self._val
            cv2.circle(frame,(self.__colorX, self.__colorY), 20, (blue,green,red), -1)
            if os.uname()[4][:3] != 'arm':
                # Setting the positions also updates HSV.
                self.__setTrackbarPosColor(self._hue, self._sat, self._val)
                self.__trackbarChanged(None)
            else:
                self.__colorsChanged(self._hue, self._sat, self._val)
            
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
        if isPyCamera == True:
            (_, contours, _) = cv2.findContours(mask.copy()
                , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            (contours, _) = cv2.findContours(mask.copy()
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
            if len(approx) == 3:
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
            cv2.imwrite('frame.jpg', frame)
            cv2.imwrite('mask.jpg', mask)
            cv2.imwrite('res.jpg', res)
            
        if self.__showWindow == True:
        # Show the frame after we annotate it.
            cv2.imshow(self._frameName, frame)
            cv2.imshow(self._maskName, mask)
            cv2.imshow(self._resName, res)
        # Return: color visible, colorX, colorY, shape visible, shapeX, shapeY, scale, count 
        targetInfo = (self.__targetColorVisible
                    , self.__targetColorX
                    , self.__targetColorY
                    , self.__targetShapeVisible
                    , self.__targetShapeX
                    , self.__targetShapeY
                    , self.__targetScale
                    , self.__targetCount)
        return targetInfo
    
    
    def __trackbarChanged(self, val):
        """
        Set the hue, saturation, and value and thresholds based on changing
        the trackpbar sliders
        """
        
        self._hue = cv2.getTrackbarPos('H', self._resName)
        self._sat = cv2.getTrackbarPos('S', self._resName)
        self._val = cv2.getTrackbarPos('V', self._resName)
    
        self._threshHue = cv2.getTrackbarPos('HT', self._resName)
        self._threshSat = cv2.getTrackbarPos('ST', self._resName)
        self._threshVal = cv2.getTrackbarPos('VT', self._resName)
    
        hue, sat, val = self._hue, self._sat, self._val
        
        if hue - self._threshHue < 0:
            hue = self._threshHue
        elif hue + self._threshHue > 255:
            hue -= 255 - hue
            
        if sat - self._threshSat < 0:
            sat = self._threshSat
        elif sat + self._threshSat > 255:
            sat -= 255 - sat
         
        if val - self._threshVal < 0:
            val = self._threshVal
        elif val + self._threshVal > 255:
            val -= 255 - val
        
        if hue != self._hue or sat != self._sat or val != self._val:
            self.__setTrackbarPosColor(hue, sat, val)
        
        print 'New HSV:', hue, sat, val
        print 'New HSV Thresholds:', self._threshHue, self._threshSat, self._threshVal
        self.__updateColorThresholds(hue, sat, val)
        return
    
    
    def __colorsChanged(self, hue, sat, val):
        """
        Use this when we are not adjusting the trackbars to change the colors. 
        """
        if hue - self._threshHue < 0:
            hue = self._threshHue
        elif hue + self._threshHue > 255:
            hue -= 255 - hue
            
        if sat - self._threshSat < 0:
            sat = self._threshSat
        elif sat + self._threshSat > 255:
            sat -= 255 - sat
         
        if val - self._threshVal < 0:
            val = self._threshVal
        elif val + self._threshVal > 255:
            val -= 255 - val
        
        print 'New HSV:', hue, sat, val
        print 'New HSV Thresholds:', self._threshHue, self._threshSat, self._threshVal
        self.__updateColorThresholds(hue, sat, val)
        return
    
    
    def __setTrackbarPosColor(self, hue, sat, val):
        """ Set the trackbar locations for HSV.
        """
        cv2.setTrackbarPos('H', self._resName, hue)
        cv2.setTrackbarPos('S', self._resName, sat)
        cv2.setTrackbarPos('V', self._resName, val)
        return
    
    
    def __setTrackbarPosThresh(self, tHue, tSat, tVal):
        """ Set the trackbar locations for HSV thresholds.
        """
        cv2.setTrackbarPos('HT', self._resName, tHue)
        cv2.setTrackbarPos('ST', self._resName, tSat)
        cv2.setTrackbarPos('VT', self._resName, tVal)
        return
        
    
    # mouse callback function
    def __mouseCallback(self, event, x, y, flags, param):
        """ Use the mouse to select colors.
        """
        if event != cv2.EVENT_MOUSEMOVE:
            # Store the image coordinates. 
            self.__colorX = x
            self.__colorY = y
            self.__getColor = True
            return
    
    
    def showImages(self, show=True):
        """ 
        """
        self.__showWindow = show
        if show == True:
            hue, sat, val = self._hue, self._sat, self._val
            tHue, tSat, tVal = self._threshHue, self._threshSat, self._threshVal
            # Conditionally start the window thread.
            cv2.startWindowThread()
            # Conditionally show windows.
            cv2.namedWindow(self._frameName, cv2.WINDOW_NORMAL)
            cv2.namedWindow(self._maskName, cv2.WINDOW_NORMAL)
            cv2.namedWindow(self._resName, cv2.WINDOW_NORMAL)
            if os.uname()[4][:3] != 'arm':
                # Filter color
                cv2.createTrackbar('H', self._resName, 0, 255, self.__trackbarChanged)
                cv2.createTrackbar('S', self._resName, 0, 255, self.__trackbarChanged)
                cv2.createTrackbar('V', self._resName, 0, 255, self.__trackbarChanged)
                # Filter color
                cv2.createTrackbar('HT', self._resName, 0, 127, self.__trackbarChanged)
                cv2.createTrackbar('ST', self._resName, 0, 127, self.__trackbarChanged)
                cv2.createTrackbar('VT', self._resName, 0, 127, self.__trackbarChanged)
                
                self.__setTrackbarPosColor(hue, sat, val)
                self.__setTrackbarPosThresh(tHue, tSat, tVal)
            
            cv2.setMouseCallback(self._frameName, self.__mouseCallback)
        return
    
    def saveImages(self):
        # Save the images the next time a picture is taken.
        self.__saveImages = True
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
        self.__rawCapture = PiRGBArray(self._cap, size=(640, 480))
        
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
        self._cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 640)
        self._cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 480)
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
        """ Close the camera device.
        """
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
        # Used to acquire an image from the camera.
        if data == 'show':
            self.__device.showImages(True)
            
        if data == 'pic':
            print 'Camera Wrapper: Got picture taking command.'
            # Single shot, save an image.
            self.__device.saveImages()
        return
    
    
    def takePicture(self):
        # Used to acquire an image from the camera.
        return self.__device.takePicture()
    
    
    def close(self):
        self.__device.close()
        return
    
    
    def showImages(self, show=True):
        self.__device.showImages(show)
        return
    
    
    
###############################################################################
# HELPER FUNCTIONS:
###############################################################################


def yuvToRgb(red, green, blue):
    green = np.uint8([[[ blue, green, red ]]])
    return cv2.cvtColor(green,cv2.COLOR_BGR2HSV)


if __name__ == "__main__":
    # Instantiate the data processor as the parent.
    cam = CameraWrapper('/dev/video0', (22, 128, 127, 10, 50, 127))
    cam.showImages(True)
    cam.readline()
    #cam.close()




