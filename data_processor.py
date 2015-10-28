import threading
from lxml import etree
import Queue, math

import ipc_manager

logger = ipc_manager.logger
#import plotter

class Configuration(object):
    
    class Port(object):
        """ For example:
        <!-- Port Configurations -->
        <port>
            <name>/dev/ttyACM0</name>
            <rate>115200</rate>
        </port>
        """
        
        def __init__(self, portNode):
            """ Port initializer.
            """
            self.__name = None
            self.__rate = None
            for child in portNode:
                if type(child) == etree._Comment: continue
                if child.tag == 'name':
                    self.__name = child.text
                if child.tag == 'rate':
                    self.__rate = int(child.text)
            return
        
        @property
        def name(self):
            return self.__name
    
        @property
        def rate(self):
            return self.__rate
        
    
    class Camera(object):
        """
        """
        
        def __init__(self, cameraNode):
            """ Camera target color initializer.
            """
            self.__name = None
            self.__properties = None
            for child in cameraNode:
                if type(child) == etree._Comment: continue
                elif child.tag == 'name':
                    self.__name = child.text
                elif child.tag == 'color':
                    for prop in child:
                        if type(prop) == etree._Comment: continue
                        elif prop.tag == 'hue':
                            self.__hue = int(prop.text)
                        elif prop.tag == 'saturation':
                            self.__saturation = int(prop.text)
                        elif prop.tag == 'value':
                            self.__value = int(prop.text)
                        elif prop.tag == 'thresh_h':
                            self.__threshHue = int(prop.text)
                        elif prop.tag == 'thresh_s':
                            self.__threshSat = int(prop.text)
                        elif prop.tag == 'thresh_v':
                            self.__threshVal = int(prop.text)
                # TODO: parse compass, accelerometer, and gyroscope settings.
            return
        
        @property
        def name(self):
            return self.__name
    
        @property
        def filterProperties(self):
            return (self.__hue, self.__saturation, self.__value
                  , self.__threshHue, self.__threshSat, self.__threshVal)
        
    
    
    class Compass(object):
        """
        """
        
        def __init__(self, compassNode):
            """ Compass calibration initializer.
            """
            self.__properties = None
            for child in compassNode:
                if type(child) == etree._Comment: continue
                elif child.tag == 'declination':
                    self.__declination = float(child.text)
                elif child.tag == 'x_north':
                    self.__xNorth = float(child.text)
                elif child.tag == 'x_east':
                    self.__xEast = float(child.text)
                elif child.tag == 'x_south':
                    self.__xSouth = float(child.text)
                elif child.tag == 'x_west':
                    self.__xWest = float(child.text)
                elif child.tag == 'y_north':
                    self.__yNorth = float(child.text)
                elif child.tag == 'y_east':
                    self.__yEast = float(child.text)
                elif child.tag == 'y_south':
                    self.__ySouth = float(child.text)
                elif child.tag == 'y_west':
                    self.__yWest = float(child.text)
                elif child.tag == 'z_north':
                    self.__zNorth = float(child.text)
                elif child.tag == 'z_east':
                    self.__zEast = float(child.text)
                elif child.tag == 'z_south':
                    self.__zSouth = float(child.text)
                elif child.tag == 'z_west':
                    self.__zWest = float(child.text)
                    
            # TODO: parse compass, accelerometer, and gyroscope settings.
            return
        
        @property
        def declination(self):
            return self.__declination
    
        # X axis compass calibration constants.
        
        @property
        def xNorth(self):
            return self.__xNorth
        
        @property
        def xEast(self):
            return self.__xEast
        
        @property
        def xSouth(self):
            return self.__xSouth
        
        @property
        def xWest(self):
            return self.__xWest
            
        # Y axis compass calibration constants.
        
        @property
        def yNorth(self):
            return self.__yNorth
        
        @property
        def yEast(self):
            return self.__yEast
        
        @property
        def ySouth(self):
            return self.__ySouth
        
        @property
        def yWest(self):
            return self.__yWest
        
        # Z axis compass calibration constants.
        
        @property
        def zNorth(self):
            return self.__zNorth
        
        @property
        def zEast(self):
            return self.__zEast
        
        @property
        def zSouth(self):
            return self.__zSouth
        
        @property
        def zWest(self):
            return self.__zWest
            
    
    def __init__(self):
        """
        Open the configuration XML file and initialize the configuration 
        subsystem:
        Mission File: mission XML file name.
        Port List: A list of TTYs for serial data streaming.
        Camera: device name, target color properties.
        Compass: declination, calibration 
        Accelerometer:
        Gyroscope:
        """
        f = open('config.xml')
        fileText = f.read()
        f.close()
        root = etree.fromstring(fileText)
        
        self.__portList = []
        self.__camera = None
        self.__compass = None
        
        for child in root:
            if type(child) == etree._Comment: continue
            elif child.tag == 'port':
                port = self.Port(child)
                self.__portList.append(port) 
            elif child.tag == 'camera':
                self.__camera = self.Camera(child)
            elif child.tag == 'compass':
                self.__compass = self.Compass(child)
            elif child.tag == 'encoder_counts_per_meter':
                self.__encoderCountsPerMeter = float(child.text)
            # TODO: parse accelerometer and gyroscope.
        return
    
    @property
    def portList(self):
        return self.__portList
    
    @property
    def camera(self):
        return self.__camera
    
    @property
    def compass(self):
        """ Fields:
        declination - magnetic declination
        xNorth   - Sensor output when oriented north
        xEast    - Sensor output when oriented east
        xSouth   - Sensor output when oriented south
        xWest    - Sensor output when oriented west
        yNorth   - ...
        yEast
        ySouth
        yWest
        zNorth
        zEast
        zSouth
        zWest
        """
        return self.__compass
        
    @property
    def encoderCountsPerMeter(self):
        """ float.
        """
        return self.__encoderCountsPerMeter
        
class MissionFile(object):
    """
    List of coordinates and their properties, target 
    """
    pass

###############################################################################
# DATA PROCESSOR: base class.
###############################################################################

class DataProcessor(object):
    """
    """
    
    # Set to false to exit message processing threads.
    RUN_SERVICE = True
    
    # Static, list of IPC managers
    __ipcManagers = []
    __processingLoopEntered = False

    # Configuration information.
    _configuration = Configuration()
    
    @property
    def gpsDataChanged(self):
        return self._gpsDataChanged
    
    
    def __init__(self, parent=True, missionPlanner=None):
        """ Data processor constructor.
        Subsystem initialization
        """ 
        if not parent:
            # Initialize some common fields used between the parent and the 
            # children, but then return immediately.
            self._txQueue = Queue.Queue()
            return
            
        logger.info("Starting IPC managers.")
        # TODO: initialize the camera manager, and get the configurations
        # from the configuration XML file.
        for port in self._configuration.portList:
            portName = port.name
            baudRate = port.rate
            self.__ipcManagers.append(ipc_manager.IpcManager(portName, baudRate))
            
        # Setup the camera IPC manager.
        cameraName = self._configuration.camera.name
        filterProperties = self._configuration.camera.filterProperties
        cameraManager = ipc_manager.IpcManager(cameraName, filterProperties)
        self.__ipcManagers.append(cameraManager)
        
        # Initialize new processors here:
        # Data processors interpret the raw (minimally processed) sensor data.
        self.__inertialNav = InertialDataProcessor(missionPlanner)
        self.__spatialNav = SpatialDataProcessor(missionPlanner)
        self.__gpsNav = GpsDataProcessor(missionPlanner)
        self.__camera = CameraDataProcessor(missionPlanner)
        
        # Message processor callbacks, used when receiving an IPC message.
        self.__processorTable = { 'i' : self.__inertialNav.processMessage
                                , 's' : self.__spatialNav.processMessage
                                , 'g' : self.__gpsNav.processMessage
                                , 'c' : self.__camera.processMessage }

        # Enqueue a message to transmit to a peripheral controller.
        self.__senderTable = { 'i' : self.__inertialNav.enqueueMessage
                             , 's' : self.__spatialNav.enqueueMessage
                             , 'g' : self.__gpsNav.enqueueMessage
                             , 'c' : self.__camera.enqueueMessage }
        
        t = threading.Thread(name='data_processor', target=self.runDataProcessors)
        # Check that the data processors have all started without issue.
        t.start()
        return
        
    def runDataProcessors(self):
        """ Poll the message loop for incoming IPC messages.
        """
        logger.info("Starting data processor.")
        __processingLoopEntered = True
        while self.RUN_SERVICE:
            for m in self.__ipcManagers:
                if not self.RUN_SERVICE: break
                # Check for None message type and skip process step.
                message = m.getMessage()
                if message is not None:
                    self.processMessage(m.getMessage())
                # Update state in the mission planner.
                
        logger.info("Stopping data processor.")
        return
    
    
    def processMessage(self, message):
        """
        """
        # Dispatch message to the relevant class's processor:
        if message is not None:
            self.__processorTable.get(message.Id, None)(message)
        return


    def shutdown(self):
        # Stop all child threads and the parent thread.
        ipc_manager.IpcManager.RUN_SERVICE = False
        DataProcessor.RUN_SERVICE = False
        return
    
    
    # IPC message queue. System controller places messages in the
    # queue to be sent to the various controllers (outgoing queue).
    def enqueueMessage(self, message):
        """ Put a message in a message queue to be read by a specific 
        data processor and transmitted to a peripheral controller via IPC.
        """
        self._txQueue.put(message)
        return
     
    
    def sendMessage(self, message):
        """ Put a message in a message queue to be read by a specific 
        data processor and transmitted to a peripheral controller via IPC.
        """
        callback = self.__senderTable.get(message[0], None)
        if callback is not None:
            callback(message)
        return
    
        
class InertialDataProcessor(DataProcessor):
    """
    Process sensor data messages in the context of a class that 'understands'
    the data and how to represent it to the mission_planner and visually via 
    some graphical display (data visualization rather that data display).
    """
    DRAW_INTERVAL = 100
    
    def __init__(self, missionPlanner):
        DataProcessor.__init__(self, False)
        self.__missionPlanner = missionPlanner
        self.__compassCalibration = self._configuration.compass
        # Take the average of the two ratios delimiting each interval except for
        # 0 / 0 which is undefined obviously.
        # From 0 (east) to pi / 2 (north)
        self.__ratio1 = (math.pi / 2.0) / self.__compassCalibration.yNorth
        # The average of north and west
        self.__ratio2 = math.pi / self.__compassCalibration.yWest
        
        self.__ratio3 = (3 * math.pi / 2.0) / self.__compassCalibration.ySouth
        
        self.__ratio3 = (2 * math.pi) / self.__compassCalibration.yEast
        
        # The average of west and south
        
        
        self.__message = None
        #self.__plotter = plotter.Plotter()
        #self.__drawCounter = 0
        # Only call this for the last data processor instantiated. 
        
        self.__compassHeadingX = 0.0
        self.__compassHeadingY = 0.0
        self.__compassHeadingZ = 0.0
        return


    def processMessage(self, message):
        # Dispatch message to the relevant class's processor:
        self.__message = message
        
        yaw = math.atan2(message.magY, message.magX)
        
        if yaw < 0.0:
            yaw += 2.0 * math.pi
            
        if yaw > 2.0 * math.pi:
            yaw -= 2.0 * math.pi
            
        yawRadians = yaw
        yawDegrees = yaw * 180.0 / math.pi
        
        #print yawRadians, yawDegrees
        
        self.__missionPlanner.updateInertial(message, yawRadians, yawDegrees)
        logger.debug(message.toString())
        
        #x = message.accX
        #y = message.accY
        #z = message.accZ
        #self.__plotter.addPoint(x, y, z, "accel")
        #if self.__drawCounter > self.DRAW_INTERVAL:
        #    self.__plotter.refreshPlot()
        #    self.__drawCounter = 0
        #else:
        #    self.__drawCounter += 1
        return
    
    
    def convertToRadians(self):
        """ Convert the magnetic sensor data to radians.
        Per the unit circle:
        east = 0
        north = pi / 2
        west = pi
        south = 3 * pi / 2
        
        Given this set of ratios, we convert the magnetic sensor value into
        radians.
        """
        return
    
    
    def convertToGravities(self):
        """ Convert accelerometer data to G's.
        """
        return
    
    
    def convertToRadiansPerSecond(self):
        """ Convert rate gyro data to radians per second (or degrees...). 
        """
        return
    
    

class SpatialDataProcessor(DataProcessor):
    """ Convert the counts to distances etc..
    """

    ID_SPACIAL = 's'
    
    def __init__(self, missionPlanner):
        DataProcessor.__init__(self, False)
        self.__missionPlanner = missionPlanner
        return
    
    
    def processMessage(self, message):
        """ Update mission planner state ...
        """
        self.__missionPlanner.updateSpacial(message)
        logger.debug(message.toString())
        # Check the outgoing message queue for spacial messages.
        if self._txQueue.qsize() > 0:
            # TODO: peek in the queue to see that it is for us...
            txMessage = self._txQueue.get()
            if txMessage[0] == self.ID_SPACIAL:
                print 'Spacial Data Processor: transmit message:', txMessage
                message.txCallback(txMessage[2:])
        return
    
    
class GpsDataProcessor(DataProcessor):
    
    def __init__(self, missionPlanner):
        DataProcessor.__init__(self, False)
        self.__missionPlanner = missionPlanner
        return
    
    def processMessage(self, message):
        # Process GPS messages. Compute northing and easting from lat/lon.
        """
        """
        lat = message.fields.latDegrees + message.fields.latMinutes / 60.0
        if message.fields.nS == 'S':
            lat *= -1
        lon = message.fields.lonDegrees + message.fields.lonMinutes / 60.0
        if message.fields.eW == 'W':
            lon *= -1
        
        if lat != self.__missionPlanner.lat or lon != self.__missionPlanner.lon:
            print lat, lon
            self.__missionPlanner.updateGps(message, lat, lon)
        
        # TODO: convert lat/lon to easting and northing (x, y) coordinates
        # in meters.
        logger.debug(message.toString())
        return


class CameraDataProcessor(DataProcessor):
    """ Camera data processor class.
    """
    
    ID_CAMERA = 'c'
    
    def __init__(self, missionPlanner):
        DataProcessor.__init__(self, False)
        self.__missionPlanner = missionPlanner
        return
    
    def processMessage(self, message):
        # Process GPS messages. Compute northing and easting from lat/lon.
        """
        """
        self.__missionPlanner.updateVision(message)
        logger.debug(message.toString())
        if self._txQueue.qsize() > 0:
            # TODO: peek in the queue to see that it is for us...
            txMessage = self._txQueue.get()
            if txMessage[0] == self.ID_CAMERA:
                print 'Camera Data Processor: transmit message:', txMessage
                message.txCallback(txMessage[2:])
        return



# Some helper functions to convert between DD MM.MM to DD.DD..
# I.e. degrees minutes to degrees  decimal degrees.
def convertLatGpsToLatMaps(latDeg, latMin, nS):
    lat = latDeg + latMin / 60.0
    if nS == 'S':
        lat *= -1
    return lat


def convertLonGpsToLonMaps(lonDeg, lonMin, eW):
    lon = lonDeg + lonMin / 60.0
    if eW == 'W':
        lon *= -1
    return lon


def convertLatDmsToLatMaps(latDeg, latMin, latSec, nS):
    lat = latDeg + latMin / 60.0 + latSec / 3600
    if nS == 'S':
        lat *= -1
    return lat


def convertLonDmsToLonMaps(lonDeg, lonMin, lonSec, eW):
    lon = lonDeg + lonMin / 60.0 + lonSec / 3600
    if eW == 'W':
        lon *= -1
    return lon

