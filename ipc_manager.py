"""
The IPC manager collects data from the various data sources, inertial, spacial,
visual, etc. It also feeds back control signals to the spacial controller in 
order to affect the platform's movement. 
"""

import threading
import time
import Queue
import serial
import logging

import cameras
        
logger = logging.getLogger()
hdlr = logging.FileHandler('system_controller.log')
formatter = logging.Formatter('%(asctime)s %(threadName)s %(levelname)s %(message)s')
hdlr.setFormatter(formatter)
logger.addHandler(hdlr) 
logger.setLevel(logging.DEBUG)

logger.info('OpenCV Version: ' + str(cameras.getCvVersion()))

class Message(object):
    """Generic message super class. Contains an ID, which indicates the data
    source, a data valid flag which indicates whether the sensor data message
    was parsed/decoded properly, creation time fields, add a callback to allow
    the data processor to send a signal/message back to the controller that 
    originated the data.
    """
    
    def __init__(self, identifier, txCallback):
        """Constructor"""
        self._creationTime = time.time() # Time since Jan 1 1970.
        self._creationTick = time.clock() # Time since start of program.
        self.__Id = identifier
        self._valid = False
        self._txCallback = txCallback
        return
    

    @property
    def valid(self):
        return self._valid
      
    @property
    def Id(self):
        return self.__Id
    
    @property
    def creationTime(self):
        return self._creationTime
    
    @property
    def creationTick(self):
        return self._creationTick
    
    @property
    def txCallback(self):
        return self._txCallback
    
    

###############################################################################
# GPS MESSAGE(s): :
###############################################################################

class GpsMessage(Message):
    """GPS Receiver - LS20031 5Hz (66 Channel)

    GGA - Global positioning system fixed data
    $GPGGA,053740.000,2503.6319,N,12136.0099,E,1,08,1.1,63.8,M,15.2,M,,0000*64
    
    Name            | Example       | Units | Description
    --------------------------------------------------------------------
    Message ID:     | $GPGGA        |       | GGA protocol header
    UTC Time:       | 053740.000    |       | hhmmss.sss
    Latitude        | 2503.6319     |       | ddmm.mmmm
    N/S indicator   | N             |       | N=north or S=south
    Longitude       | 12136.0099    |       | dddmm.mmmm
    E/W Indicator   | E             |       | E=east or W=west
    Position Fix    | 1             |       | See Table 5.1-3
    Satellites Used | 08            |       | Range 0 to 12
    HDOP            | 1.1           |       | Horizontal Dilution of Precision
    MSL Altitude    | 63.8          | meters| 
    Units           | M             | meters|
    Geoid Separation| 15.2          | meters|
    Units           | M             | meters|
    Age of Diff. Corr. |            | second| Null fields when DGPS is not used
    Diff. Ref. Station ID | 0000    |       |
    Checksum        | *64           |       |
    <CR> <LF>       |               |       | End of message termination

    Table 5.1-3:
        0   - Fix not available or invalid
        1   - GPS SPS Mode, fix valid
        2   - Differential GPS, SPS Mode, fix valid
        3-5 - Not supported
        6   - Dead Reckoning Mode, fix valid

    ________________________________________________________________________

    GLL - Geographic position - latitude/longitude
    $GPGLL,2503.6319,N,12136.0099,E,053740.000,A,A*52

    Name            | Example       | Units | Description
    --------------------------------------------------------------------
    Message ID:     | $GPGLL        |       | GLL protocol header
    Latitude        | 2503.6319     |       | ddmm.mmmm
    ...

    GSA - GNSS DOP and active satellites
    GSV - GNSS satelites in view
    RMC - Recommended minimum specific GNSS data
    VTG - Course over ground and ground speed
    """
    
    GPS_GGA = 'GPGGA'
    GPS_GLL = 'GPGLL'
    GPS_GSV = 'GPGSV'
    GPS_RMC = 'GPRMC'
    GPS_VTG = 'GPVTG'
    
    class GgaData(object):
        """Initialize a GGA message."""
        
        def __init__(self, data):
            """Constructor"""
            # Name            | Example       | Units | Description
            # --------------------------------------------------------------------
            # Message ID:     | $GPGGA        |       | GGA protocol header
            self.__Id   = data[0]
            # UTC Time:       | 053740.000    |       | hhmmss.sss
            self.__utc  = data[1]
            self.__utcHours = int(self.__utc[:2])
            self.__utcMinutes = int(self.__utc[2:4])
            self.__utcSeconds = float(self.__utc[4:])
            # Latitude        | 2503.6319     |       | ddmm.mmmm
            self.__lat  = data[2]
            dddmm, mmm = self.__lat.split('.')
            self.__latDegrees = int(dddmm[:-2])
            self.__latMinutes = float(dddmm[-2:] + '.' + mmm)
            # N/S indicator   | N             |       | N=north or S=south
            self.__nS   = data[3]
            # Longitude       | 12136.0099    |       | dddmm.mmmm
            self.__lon  = data[4]
            dddmm, mmm = self.__lon.split('.')
            self.__lonDegrees = int(dddmm[:-2])
            self.__lonMinutes = float(dddmm[-2:] + '.' + mmm)
            # E/W Indicator   | E             |       | E=east or W=west
            self.__eW   = data[5]
            # Position Fix    | 1             |       | See Table 5.1-3
            self.__fix  = int(data[6])
            # Satellites Used | 08            |       | Range 0 to 12
            self.__sat  = int(data[7])
            # HDOP            | 1.1           |       | Horizontal Dilution of Precision
            self.__hdop = float(data[8])
            # MSL Altitude    | 63.8          | meters| 
            self.__alt  = float(data[9])
            # Geoid Separation| 15.2          | meters|
            self.__sep  = float(data[11])
            # Age of Diff. Corr. |            | second| Null fields when DGPS is not used
            self.__age  = data[13]
            # Checksum        | *64           |       |
            self.__ref  = data[14]
            return
        
        
        @property
        def Id(self):
            """Different than the message ID, this is the GPS message ID."""
            return self.__Id
        
        @property
        def utc(self):
            return self.__utc
        
        @property
        def utcHours(self):
            return self.__utcHours
        
        @property
        def utcMinutes(self):
            return self.__utcMinutes
        
        @property
        def utcSeconds(self):
            return self.__utcSeconds
        
        @property
        def lat(self):
            return self.__lat
        
        @property
        def latDegrees(self):
            return self.__latDegrees
        
        @property
        def latMinutes(self):
            return self.__latMinutes
        
        @property
        def nS(self):
            return self.__nS
        
        @property
        def lon(self):
            return self.__lon
        
        @property
        def lonDegrees(self):
            return self.__lonDegrees
        
        @property
        def lonMinutes(self):
            return self.__lonMinutes
        
        @property
        def eW(self):
            return self.__eW
        
        @property
        def fix(self):
            return self.__fix
        
        @property
        def sat(self):
            return self.__sat
        
        @property
        def hdop(self):
            return self.__hdop
        
        @property
        def alt(self):
            return self.__alt
        
        @property
        def sep(self):
            return self.__sep
        
        @property
        def age(self):
            return self.__age
        
        @property
        def ref(self):
            return self.__ref
        
        
    
    __messageTable = { GPS_GGA : GgaData }
    
    ID_GPS = 'g'
    
    def __init__(self, data, txCallback):
        """Initialize the GPS message."""
        
        # Look up a GPS message type class to instantiate.
        data = data.strip()[1:].split(',')
        message = self.__messageTable.get(data[0], None)
        if message is not None:
            Message.__init__(self, self.ID_GPS, txCallback)
            try:
                self.__fields = message(data)
                self.toString()
            except Exception, e:
                self.__fields = None
                logger.warning('GPS parse error: ' + str(e))
        else:
            # We dont decode this NMEA string yet...
            Message.__init__(self, None, None)
            self.__fields = -1
        self._valid = self.__fields is not None
        return
    
    
    def toString(self):
        """Turn the message into its string representation."""
        c = ','
        iString = c + self.ID_GPS + c + str(self.__fields.Id) + c \
            + str(self.__fields.utc) + c + str(self.__fields.lat) + c \
            + str(self.__fields.nS) + c + str(self.__fields.lon) + c \
            + str(self.__fields.eW) + c + str(self.__fields.fix) + c \
            + str(self.__fields.sat) + c + str(self.__fields.hdop) + c \
            + str(self.__fields.alt) + c + str(self.__fields.sep) + c \
            + str(self._creationTime) + c + str(self._creationTick)
        return iString
    
    
    @property
    def fields(self):
        return self.__fields
    
    


###############################################################################
# INERTIAL MESSAGE: direction, acceleration, rotation:
###############################################################################

class InertialMessage(Message):
    """ Parse a message from the IMU. Someday, if it is feasable, we will
    delegate all the IMU algorithms to the embedded controller, then we will
    also get: compass heading, speed, distance, pitch, roll from accelerometer,
    and pitch, roll, yaw, from rate-gyro. 
    """
    
    ID_INERTIAL = 'i'
    IN_MOTION = 1
    ACCEL_GYRO_CAL = 2
    COMPASS_CAL = 4
    
    def __init__(self, data, txCallback):
        """InertialMessage constructor."""
        #if not data.startswith('$GPGGA'): break 
        Message.__init__(self, self.ID_INERTIAL, txCallback)
        data = data.strip()[2:].split(',')
        # Raw sensor data:
        self.__magX = float(data[0])
        self.__magY = float(data[1])
        self.__magZ = float(data[2])
        self.__accX = int(data[3])
        self.__accY = int(data[4])
        self.__accZ = int(data[5])
        self.__gyroX = float(data[6])
        self.__gyroY = float(data[7])
        self.__gyroZ = float(data[8])
        # Signal conditioned data:
        # Adjusted magnetic field data X
        self.__adjMagX = float(data[9])
        # Adjusted magnetic field data Y
        self.__adjMagY = float(data[10])
        # Heading
        self.__heading = float(data[11])
        # Accelerometer pitch
        self.__accPitch = float(data[12])
        # Accelerometer roll
        self.__accRoll = float(data[13])
        # Accelerometer speed X (integrated)
        self.__accSpeedX = float(data[14])
        # Accelerometer speed Y (integrated)
        self.__accSpeedY = float(data[15])
        # Accelerometer speed Z (integrated)
        self.__accSpeedZ = float(data[16])
        # Accelerometer distance X (integrated)
        self.__accDistX = float(data[17])
        # Accelerometer distance Y (integrated)
        self.__accDistY = float(data[18])
        # Accelerometer distance Z (integrated)
        self.__accDistZ = float(data[19])
        # Gyroscope degrees rotated X
        self.__gyroDegreesX = float(data[20])
        # Gyroscope degrees rotated Y
        self.__gyroDegreesY = float(data[21])
        # Gyroscope degrees rotated Z
        self.__gyroDegreesZ = float(data[22])
        # Status field
        # Bit 0: in motion
        # Bit 1: accelerometer and gyroscope are calibrated
        # Bit 2: compass is calibrated
        self.__status = int(data[23])
        self.__inMotion = bool(self.__status & self.IN_MOTION)
        self.__accelGyroCal = bool(self.__status & self.ACCEL_GYRO_CAL)
        self.__compassCal = bool(self.__status & self.COMPASS_CAL)
        self.__milliseconds = int(data[24])
        self.__microseconds = int(data[25])
        self._valid = True
        return
    
    
    def toString(self):
        """Turn the inertial measurement message into a string."""
        c = ','
        """
            # Don't include the raw values in the string.
            + str(self.__magY) + c + str(self.__magZ) + c \
            + str(self.__accX) + c + str(self.__accY) + c \
            + str(self.__accZ) + c + str(self.__gyroX) + c \
            + str(self.__gyroY) + c + str(self.__gyroZ) + c \
        """
        iString = c + self.ID_INERTIAL + c + str(self.__magX) + c \
            + str(self.__adjMagX) + c + str(self.__adjMagY) + c \
            + str(self.__heading) + c + str(self.__accPitch) + c \
            + str(self.__accRoll) + c + str(self.__accSpeedX) + c \
            + str(self.__accSpeedY) + c + str(self.__accSpeedZ) + c \
            + str(self.__accDistX) + c + str(self.__accDistY) + c \
            + str(self.__accDistZ) + c + str(self.__gyroDegreesX) + c \
            + str(self.__gyroDegreesY) + c + str(self.__gyroDegreesZ) + c \
            + str(self.__status) + c + str(self.__inMotion) + c \
            + str(self.__accelGyroCal) + c + str(self.__compassCal) + c \
            + str(self.__milliseconds) + c + str(self.__microseconds) + c \
            + str(self._creationTime) + c + str(self._creationTick)
        return iString
    
    
    @property
    def magX(self):
        return self.__magX
    
    @property
    def magY(self):
        return self.__magY
    
    @property
    def magZ(self):
        return self.__magZ
    
    @property
    def accX(self):
        return self.__accX
    
    @property
    def accY(self):
        return self.__accY
    
    @property
    def accZ(self):
        return self.__accZ
    
    @property
    def gyroX(self):
        return self.__gyroX
    
    @property
    def gyroY(self):
        return self.__gyroY
    
    @property
    def gyroZ(self):
        return self.__gyroZ
    
    @property
    def adjMagX(self):
        return self.__adjMagX
    
    @property
    def adjMagY(self):
        return self.__adjMagY
    
    @property
    def heading(self):
        return self.__heading
    
    @property
    def accPitch(self):
        return self.__accPitch
    
    @property
    def accRoll(self):
        return self.__accRoll
    
    @property
    def accSpeedX(self):
        return self.__accSpeedX
    
    @property
    def accSpeedY(self):
        return self.__accSpeedY
    
    @property
    def accSpeedZ(self):
        return self.__accSpeedZ
    
    @property
    def accDistX(self):
        return self.__accDistX
    
    @property
    def accDistY(self):
        return self.__accDistY
    
    @property
    def accDistZ(self):
        return self.__accDistZ
    
    @property
    def gyroDegreesX(self):
        return self.__gyroDegreesX
    
    @property
    def gyroDegreesY(self):
        return self.__gyroDegreesY
    
    @property
    def gyroDegreesZ(self):
        return self.__gyroDegreesZ
    
    @property
    def status(self):
        return self.__status
    
    @property
    def inMotion(self):
        return self.__inMotion
    
    @property
    def accelGyroCal(self):
        return self.__accelGyroCal
    
    @property
    def compassCal(self):
        return self.__compassCal
    
    @property
    def milliseconds(self):
        return self.__milliseconds
    
    @property
    def microseconds(self):
        return self.__microseconds
    
    

###############################################################################
# SPACIAL NAVIGATION MESSAGE(s): :
###############################################################################

class SpacialMessage(Message):
    """Message parser for the spacial navigation controller."""
    
    ID_SPACIAL = 's'
    
    def __init__(self, data, txCallback):
        """Constructor."""
        # TODO: enable debug messages from the microcontroller, something like:
        # if data starts with 'DEBUG:' print message in the log, then return.
        Message.__init__(self, self.ID_SPACIAL, txCallback)
        data = data.strip()[2:].split(',')
        # Parse sonar sensor data:
        self.__sonarLeft = float(data[0])
        self.__sonarFront = float(data[1])
        self.__sonarRight = float(data[2])
        self.__sonarBack = float(data[3])
        # Parse infrared sensor data:
        self.__irDown  = float(data[4])
        #self.__irLeft  = data[5]
        #self.__irRight = data[6]
        #self.__irBack  = data[7]
        # Encoder counter and motor status (faults):
        self.__encoderCountsLeft  = int(data[5])
        self.__encoderCountsRight = int(data[6])
        # Left motor status:
        self.__motorStatusLeft = int(data[7])
        self.__motorControlVariableLeft = self.__motorStatusLeft & 0xFF
        self.__motorSetPointLeft = (self.__motorStatusLeft >> 8) & 0xFF
        self.__motorProcessVariableLeft = (self.__motorStatusLeft >> 16) & 0xFF
        self.__motorsEnabled = bool((self.__motorStatusLeft >> 24) & 0xFF)
        # Right motor status:
        self.__motorStatusRight = int(data[8])
        self.__motorControlVariableRight = self.__motorStatusRight & 0xFF
        self.__motorSetPointRight = (self.__motorStatusRight >> 8) & 0xFF
        self.__motorProcessVariableRight = (self.__motorStatusRight >> 16) & 0xFF
        self.__motorsRunMode = (self.__motorStatusRight >> 24) & 0xFF
        # TODO: extract the run mode and motors enabled status from motor status.
        # Remote control buttons:
        self.__buttonA = int(data[9])
        self.__buttonB = int(data[10])
        self.__buttonC = int(data[11])
        self.__buttonD = int(data[12])
        # Timestamps from the uController
        self.__milliseconds = int(data[13])
        self.__microseconds = int(data[14])
        self._valid = True
        return
    
    
    def toString(self):
        """Turn the spacial message into a string."""
        c = ','
        iString = c + self.ID_SPACIAL + c + str(self.__sonarLeft) + c \
            + str(self.__sonarFront) + c + str(self.__sonarRight) + c \
            + str(self.__sonarBack) + c + str(self.__irDown) + c \
            + str(self.__encoderCountsLeft) + c \
            + str(self.__encoderCountsRight) + c \
            + str(self.__motorStatusLeft) + c \
            + str(self.__motorStatusRight) + c \
            + str(self.__buttonA) + c + str(self.__buttonB) + c \
            + str(self.__buttonC) + c + str(self.__buttonD) + c \
            + str(self.__milliseconds) + c + str(self.__microseconds) + c \
            + str(self._creationTime) + c + str(self._creationTick)
        return iString
    
    
    # Sonar sensor data:
    @property
    def sonarLeft(self):
        return self.__sonarLeft
    
    @property
    def sonarFront(self):
        return self.__sonarFront
    
    @property
    def sonarRight(self):
        return self.__sonarRight
    
    @property
    def sonarBack(self):
        return self.__sonarBack
    
    # Infrared sensor data:
    @property
    def irDown(self):
        return self.__irDown
    
    @property
    def irLeft(self):
        return self.__irLeft
    
    @property
    def irRight(self):
        return self.__irRight
    
    @property
    def irBack(self):
        return self.__irBack
    
    # Encoder counter and motor status (faults):
    @property
    def encoderCountsLeft(self):
        return self.__encoderCountsLeft
    
    @property
    def encoderCountsRight(self):
        return self.__encoderCountsRight
    
    @property
    def motorStatusLeft(self):
        return self.__motorStatusLeft
    
    @property
    def motorControlVariableLeft(self):
        return self.__motorControlVariableLeft
    
    @property
    def motorSetPointLeft(self):
        return self.__motorSetPointLeft
    
    @property
    def motorProcessVariableLeft(self):
        return self.__motorProcessVariableLeft
    
    @property
    def motorsEnabled(self):
        return self.__motorsEnabled
    
    @property
    def motorStatusRight(self):
        return self.__motorStatusRight
    
    @property
    def motorControlVariableRight(self):
        return self.__motorControlVariableRight
    
    @property
    def motorSetPointRight(self):
        return self.__motorSetPointRight
    
    @property
    def motorProcessVariableRight(self):
        return self.__motorProcessVariableRight
    
    @property
    def motorsRunMode(self):
        return self.__motorsRunMode
    
    @property
    def buttonA(self):
        return self.__buttonA
    
    @property
    def buttonB(self):
        return self.__buttonB
    
    @property
    def buttonC(self):
        return self.__buttonC
    
    @property
    def buttonD(self):
        return self.__buttonD
    
    @property
    def milliseconds(self):
        return self.__milliseconds
    
    @property
    def microseconds(self):
        return self.__microseconds
    
    

###############################################################################
# CAMERA MESSAGE: target in view, target coordinates:
###############################################################################

class CameraMessage(Message):
    """Camera message class."""
    
    ID_CAMERA = 'c'
    
    def __init__(self, data, txCallback):
        """Camera message constructor"""
        Message.__init__(self, self.ID_CAMERA, txCallback)
        if data[1] is not None:
            self.__targtColorVisible = data[1][0]
            self.__targetColorX = data[1][1]
            self.__targetColorY = data[1][2]
            self.__targetShapeVisible = data[1][3]
            self.__targetShapeX = data[1][4]
            self.__targetShapeY = data[1][5]
            self.__targetScale = data[1][6]
            self.__targetCount = data[1][7]
            # Also get the image data if it is avaliable.
            self.__imageData = data[1][8]
            self._valid = True
        else:
            self._valid = False
        #time.sleep(0.025) # Sleep here to control frame rate.
        return
    
    
    def toString(self):
        """Turn the camera message into a string."""
        c = ','
        iString = c + self.ID_CAMERA \
            + c + str(self.__targtColorVisible) \
            + c + str(self.__targetColorX) \
            + c + str(self.__targetColorY) \
            + c + str(self.__targetShapeVisible) \
            + c + str(self.__targetShapeX) \
            + c + str(self.__targetShapeY) \
            + c + str(self.__targetScale) \
            + c + str(self.__targetCount) \
            + c + str(self._creationTime) + c + str(self._creationTick)
        return iString
    
    
    @property
    def targtColorVisible(self):
        return self.__targtColorVisible
    
    @property
    def targetColorX(self):
        return self.__targetColorX
    
    @property
    def targetColorY(self):
        return self.__targetColorY
    
    @property
    def targetShapeVisible(self):
        return self.__targetShapeVisible
    
    @property
    def targetShapeX(self):
        return self.__targetShapeX
    
    @property
    def targetShapeY(self):
        return self.__targetShapeY
    
    @property
    def targetScale(self):
        return self.__targetScale
    
    @property
    def targetCount(self):
        return self.__targetCount
    
    

###############################################################################
# IPC MANAGER :
###############################################################################

class IpcManager(object):
    """Class IPC manager, polls the controllers for data."""

    __runService = True;

    # Message type ID's:
    INERTIAL_NAV = 'i'
    SPATIAL_NAV = 's'
    GPS_NAV = '$'
    CAMERA = 'c'
    
    # The callback table contains function pointers that allow the correct
    # message to be instantiated upon receipt of a new message.
    __callbackTable = { INERTIAL_NAV : InertialMessage
                      , SPATIAL_NAV : SpacialMessage
                      , GPS_NAV : GpsMessage 
                      , CAMERA : CameraMessage }
    
    __messageQueue = Queue.Queue()
    
    def __init__(self, port, configuration):
        # Open the designated communications channel:
        # Optionally instantiate a camera...
        if type(configuration) != tuple:
            # For the serial port, all we need is its name and baudrate.
            try:
                baudrate = configuration
                self.__channel = serial.Serial(port, baudrate, timeout=1)
                if not self.__channel.isOpen():
                    message = "Can't open port: " + str(port), '. Aborting thread.'
                    logger.critical(message)
                    print message
                    return
            except Exception, e:
                message = "Can't open port: " + str(e), '. Aborting thread.'
                logger.critical(message)
                print message
                return
        else:
            self.__channel = cameras.CameraWrapper(port, configuration)
        # Add some identifying information from the init parameters.
        t = threading.Thread(name='ipc_manager' + port, target=self.reader)
        t.daemon = True
        if self.__channel.isOpen():
            # Only start the reader thread if the port is open.
            t.start()
        return
    
    
    def shutdown(self):
        self.__runService = False
        return
    
    
    def reader(self):
        """Read from the peripheral processor and enqueue its messages. We 
        also use this for image acquisition from a camera device. The idiom
        is: readline == take picture.
        """
        logger.info('Starting')
        
        while self.__runService:
            data = self.__channel.readline()
            if len(data) == 0:
                logger.info('Read timeout')
                continue
            # Get the message constructor based on the message type.
            callback = self.__callbackTable.get(data[0], None)
            if callback is not None:
                instance = callback(data, self.putMessage)
                if instance.valid:
                    self.__messageQueue.put(instance)
                else:
                    message = 'Received invalid message: ' + repr(data)
                    logger.info(message)
                    print message
            else:
                if not repr(data).startswith("'Received command:"):
                    message = 'Received unknown message: ' + repr(data)
                    logger.info(message)
                    print message
        
        logger.info('IPC thread __runService = False, closing channel.')
        self.__channel.close()
        logger.info('Channel closed')
        queueSize = self.__messageQueue.qsize()
        message = 'Messages in queue at exit: ' + str(queueSize)
        logger.info(message)
        print message
        # End of thread's run loop.
        return
    
    
    def messageAvaliable(self):
        """Predicate flag to indicate that the message queue contains messages.
        """
        return not self.__messageQueue.empty()
    
    
    def getMessage(self):
        """Dequeue a message."""
        #print self.__messageQueue.qsize()
        try:
            return self.__messageQueue.get(True, 10)
        except Exception, e:
            print e
        return
    
            
    def putMessage(self, data):
        """Enqueue a message to send to a controller. Used as the txCallback
        in message constructors."""
        #print self.__messageQueue.qsize()
        try:
            self.__channel.write(data)
        except Exception, e:
            print e
        return
    
    
