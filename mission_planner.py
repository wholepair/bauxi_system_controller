"""
The mission planner interprets mission directives and state information
in order to execute the robot's mission.

Directives include: way-points (mandatory and optional)

Way-point Descriptors: navigation-point, target-point, finish-point, etc..

Sequencing: order to achieve way-points

Behaviors: what to do at target way-point, contact, dwell, repetition:
           to achieve way-points a single time only or in a loop.


TODO: Implement calibration state machine and process.
"""

import sys, math, time
from lxml import etree
import utm
import matplotlib.pyplot as plt

import data_processor
from data_processor import DataProcessor
from static_maps import StaticMap

#from debug_view import Ui_MainWindow

logger = data_processor.logger

LOG_LEVEL_INFO     = 0
LOG_LEVEL_WARNING  = 1
LOG_LEVEL_ERROR    = 2
LOG_LEVEL_CRITICAL = 3

LOG_LEVEL_NAMES = { LOG_LEVEL_INFO     : 'LOG_LEVEL_INFO'
                  , LOG_LEVEL_WARNING  : 'LOG_LEVEL_WARNING'
                  , LOG_LEVEL_ERROR    : 'LOG_LEVEL_ERROR' 
                  , LOG_LEVEL_CRITICAL : 'LOG_LEVEL_CRITICAL' }

def logPrint(message, printMessage=True, level=0):
    if level == LOG_LEVEL_INFO:
        logger.info(message)
    elif level == LOG_LEVEL_WARNING:
        logger.warning(message)
    elif level == LOG_LEVEL_ERROR:
        logger.error(message)
    elif level == LOG_LEVEL_CRITICAL:
        logger.critical(message)
    
    if printMessage:
        print LOG_LEVEL_NAMES[level] + ':\t' + message
    return


class TimeoutTimer(object):
    
    def __init__(self, duration = 0):
        """Initialize the duration field. Duration is in tens of seconds, 
        floating point"""
        self.resetTimeout(duration)
    
        
    def checkTimeout(self):
        """Return the number of of seconds until timeout.
        If the duration has not been elapsed, return a positive number, 
        otherwise the return value is negative."""
        #print self.__timeout - time.clock()
        return self.__timeout - time.clock()
    
    
    def resetTimeout(self, duration):
        """Reset the timer to some time further into the future."""
        self.__startTime = time.clock() # Time since start of program.
        self.__timeout = self.__startTime + duration
        self.__duration = duration
        

class MissionLocations(object):
    """Things to remember:
    Remove the xmlns attribute, it breaks tag string compare operations
    - Add type and handle data to the rtept point list.
    - The 'type' attribute is new and different than Sputnik's 'note' attribute.
      (but it is uesd for more or less the same purpose).
    - 
    """
    
    def __init__(self, fileName):
        """Constructor"""
        # Open the mission file, more or less a GPX or KML file
        self.__locationsLatLon = []
        self.__locationsUtm = []
        f = open(fileName)
        fileText = f.read()
        f.close()
        root = etree.fromstring(fileText)
        
        for child in root:
            if type(child) == etree._Comment: continue
            elif child.tag == 'gpx':
                root = child
                break
                
        for child in root:
            if type(child) == etree._Comment: continue
            elif child.tag == 'metadata':
                for field in child:
                    if type(field) == etree._Comment: continue
                    elif field.tag == 'name':
                        self.__metaName = field.text
                    elif field.tag == 'desc':
                        self.__metaDesc = field.text
            elif child.tag == 'rte':
                logPrint('Mission File Locations:')
                for field in child:
                    if type(field) == etree._Comment: continue
                    elif field.tag == 'name':
                        self.__rteName = field.text
                    elif field.tag == 'desc':
                        self.__rteDesc = field.text
                    elif field.tag == 'rtept':
                        lat = float(field.get('lat'))
                        lon = float(field.get('lon'))
                        locType = field.get('type')
                        handle = field.get('handle')
                        latLonLocation = (lat, lon, locType, handle)
                        logPrint('Lat/Lon Location: ' + repr(latLonLocation))
                        self.__locationsLatLon.append(latLonLocation)
                        utmLocation = utm.from_latlon(lat, lon)
                        self.__locationsUtm.append((utmLocation, locType, handle))
                        logPrint('UTM Location: ' + repr(utmLocation))
        return
    
    
    @property
    def locationsUtm(self):
        """Locations converted from Lant/Lon to UTM."""
        return self.__locationsUtm
    
    @property
    def locationsLatLon(self):
        """Lat/Lon locations."""
        return self.__locationsLatLon
    
    @property
    def rteName(self):
        """The route name, influences the name of the map image."""
        return self.__rteName
    
    

class MissionPlanner(object):
    """High level control of the robot is implemented here."""
    # Mission planner state list:
    # States are used for internal program control flow, not externally visible.
    STATE_UNKNOWN     = -1 # Undefined initial state.
    STATE_STARTUP     = 0 # State used when bringing up subsystems.
    STATE_SELF_TEST   = 1 # State used when testing subsystems.
    STATE_NOMINAL     = 2 # All self tests passed.
    STATE_FAILED      = 3 # Used when one or more critical subsystems fail.
    STATE_IMPAIRED    = 4 # Used when a non-critical subsystem is down.
    STATE_SHUTDOWN    = 5 # 
    
    STATE_NAMES = { STATE_UNKNOWN     : 'STATE_UNKNOWN'
                  , STATE_STARTUP     : 'STATE_STARTUP'
                  , STATE_SELF_TEST   : 'STATE_SELF_TEST'
                  , STATE_NOMINAL     : 'STATE_NOMINAL'
                  , STATE_FAILED      : 'STATE_FAILED'
                  , STATE_IMPAIRED    : 'STATE_IMPAIRED'
                  , STATE_SHUTDOWN    : 'STATE_SHUTDOWN' }
    
    __currentState = STATE_UNKNOWN
    logPrint('Setting current state to: STATE_UNKNOWN.')
    
    # Normal operating modes, may also be used in the development state:
    # Modes are externally visible behavior modes. 
    MODE_UNKNOWN = -1
    MODE_WAYPOINT_SEARCH = 0 # Use vector navigation and GPS, move to waypoint. 
    MODE_TARGET_SEARCH = 1   # Looking for a target object (visual search)
    MODE_TARGET_TRACKING = 2 # Target is visually acquired, track it.
    MODE_CALIBRATION = 3     # Calibrate inertial measurement subsystem.
    MODE_AUTO_AVOID = 4      # An obstacle is detected, allow subsystem to control
    MODE_REMOTE_CONTROL = 5  # Control the robot with a remote control
    MODE_HALT = 6            # Stop all motion
    
    MODE_NAMES = { MODE_UNKNOWN         : 'MODE_UNKNOWN'
                 , MODE_WAYPOINT_SEARCH : 'MODE_WAYPOINT_SEARCH'
                 , MODE_TARGET_SEARCH   : 'MODE_TARGET_SEARCH'
                 , MODE_TARGET_TRACKING : 'MODE_TARGET_TRACKING'
                 , MODE_CALIBRATION     : 'MODE_CALIBRATION'
                 , MODE_AUTO_AVOID      : 'MODE_AUTO_AVOID'
                 , MODE_REMOTE_CONTROL  : 'MODE_REMOTE_CONTROL'
                 , MODE_HALT            : 'MODE_HALT' }
    
    __currentMode = MODE_UNKNOWN
    logPrint('Setting current mode to: MODE_UNKNOWN.')
    
    # Spacial controllers modes of operation, see
    SPACIAL_MODE_UNSET = -1
    SPACIAL_MODE_REMOTE_CONTROL  = 0 # Rely on remote (radio) control for movement.
    SPACIAL_MODE_SYS_CONTROL     = 1 # Rely on external software control commands.
    SPACIAL_MODE_AUTO_AVOID      = 2 # Use sensors to automatically avoid objects.
    SPACIAL_MODE_MANUAL_OVERRIDE = 3 # Manual override radio dead-man control.
    
    SPACIAL_MODE_NAMES = { SPACIAL_MODE_UNSET           : 'SPACIAL_MODE_UNSET'
                         , SPACIAL_MODE_REMOTE_CONTROL  : 'SPACIAL_MODE_REMOTE_CONTROL'
                         , SPACIAL_MODE_SYS_CONTROL     : 'SPACIAL_MODE_SYS_CONTROL'
                         , SPACIAL_MODE_AUTO_AVOID      : 'SPACIAL_MODE_AUTO_AVOID'
                         , SPACIAL_MODE_MANUAL_OVERRIDE : 'SPACIAL_MODE_MANUAL_OVERRIDE'
                         }
    
    __currentSpacialMode = SPACIAL_MODE_UNSET
    
    # Modes of the radio, hand controller. aka dead-man switch
    RADIO_MODE_UNSET  = -1
    RADIO_MODE_MOTION_DISABLED  = 0
    RADIO_MODE_MOTION_ENABLED_AUTO = 1
    RADIO_MODE_MOTION_ENABLED_REMOTE = 2
    
    RADIO_MODE_NAMES = { RADIO_MODE_UNSET                 : 'RADIO_MODE_UNSET'
                       , RADIO_MODE_MOTION_DISABLED       : 'RADIO_MODE_MOTION_DISABLED'
                       , RADIO_MODE_MOTION_ENABLED_AUTO   : 'RADIO_MODE_MOTION_ENABLED_AUTO'
                       , RADIO_MODE_MOTION_ENABLED_REMOTE : 'RADIO_MODE_MOTION_ENABLED_REMOTE'
                         }
    
    THRESHOLD_TARGET_DISTANCE = 15 # Meters to target before we look for it...
    THRESHOLD_WAYPOINT_DISTANCE = 5 # Meters to location before we go to the next one.
    
    TURN_LEFT = 0
    TURN_RIGHT = 1
    TURN_STRAIGHT = 2
    
    TURN_DIRECTION_NAMES = { TURN_LEFT     : 'TURN_LEFT'
                           , TURN_RIGHT    : 'TURN_RIGHT'
                           , TURN_STRAIGHT : 'TURN_STRAIGHT'
                           }
    
    SPEED_ULTRA = 200
    SPEED_MAX = 160
    SPEED_MED = 150
    SPEED_MIN = 140
    SPEED_STOP = 128
    
    # TODO: tune these ratios
    TURN_RATIO = float(2 * (SPEED_MED - SPEED_STOP)) / 180.0
    
    CAMERA_HORIZONTAL_RESOLUTION = 640
    CAMERA_HORIZONTAL_CENTER = CAMERA_HORIZONTAL_RESOLUTION / 2.0
    # TODO: get the horizontal resolution from the camera implementation.
    TURN_RATIO_VISION = float(2 * (SPEED_MED - SPEED_STOP)) / CAMERA_HORIZONTAL_CENTER
    
    SPEED_UPDATE_INTERVAL = 1 # Frequency of speed feedback to motor controller.
    
    LOCATION_TYPE_START = 'start'
    LOCATION_TYPE_WAYPOINT = 'waypoint'
    LOCATION_TYPE_TARGET = 'target'
    LOCATION_TYPE_FINISH = 'finish' # Don't use this one. Why not?
    
    # Number of times all systems are not on line when processing a spacial 
    # message before we are considered to be in a failed state.
    FAILED_STATE_COUNT_LIMIT = 100
    
    # Search patterns
    SEARCH_PATTERN_NA = -1
    SEARCH_PATTERN_1 = 0
    SEARCH_PATTERN_2 = 1
    SEARCH_PATTERN_3 = 2
    
    SEARCH_PATTERN_NAMES = { SEARCH_PATTERN_NA : "SEARCH_PATTERN_NA"
                           , SEARCH_PATTERN_1  : 'SEARCH_PATTERN_1'
                           , SEARCH_PATTERN_2  : 'SEARCH_PATTERN_2'
                           , SEARCH_PATTERN_3  : 'SEARCH_PATTERN_3'
                           }
    
    __currentSearchPattern = SEARCH_PATTERN_NA
    TARGET_SEARCH_DISTANCE_LIMIT = 20.0 # Meters.
    TARGET_SCALE_THRESH = 300.0
    AUTO_AVOID_DEBOUNCE_COUNT = 12
    
    __staticMap = StaticMap()
    
    def __init__(self):
        """ Constructor
        The target location list is a list of locations the robot must go.
        Start, location 1, 2, 3, etc..
        Format: name: (lat, lon), type
        The location type specifies what we must do once we get to the location.
        The location types are: start, waypoint, target, and finish.
        The start location type is the starting location.
        The waypoint location type is an intermediate location, used to 
        direct the robot away from hazards, etc..
        """
        self.__shutdownRequested = False
        self.__shutdownCountdown = 5
        self.__autoAvoidCountdown = self.AUTO_AVOID_DEBOUNCE_COUNT
        
        # The distance to the next/current location in the location list.
        self.__distanceToLocation = sys.maxint
        self.__distanceFromLastTarget = 0
        # A boolean flag, True when the vision system has spotted the target.
        self.__targetShapeVisible = False
        self.__targetShapeLocked = False
        self.__targetShapeX = 0
        self.__targetShapeY = 0
        self.__targetScale = 0
        # The distance to the target, measured by the range sensor
        self.__distanceToTarget = -1.0
        self.__failedStateCounter = 0
        self.__spacialUpdateCounter = 0
        
        # How far we have driven in visual search of the target object.
        self.__targetSearchDistance = 0
        
        self.__utmGps = ()
        self.__utmVector = ()
        
        self.__encoderCountsLeft = 0
        self.__encoderCountsRight = 0
        self.__prevEncoderCountsLeft = 0
        self.__prevEncoderCountsRight = 0
        self.__radioMode = self.RADIO_MODE_UNSET
        self.__loopDisplacement = 0 # How far we moved this iteration.
        self.__vectorDisplacement = 0;
        
        self.__averageHeading = 0.0
        self.__prevAverageHeading = -1.0
        self.__headingList = []
        self.__crosstrackError = 0
        
        self.__vectorPositionX = 0 # Position in meters, relative to the starting
        self.__vectorPositionY = 0 # UTM position or an intermediate waypoint.
        
        self.__messageSpacial = None
        self.__messageInertial = None
        self.__messageGps = None
        self.__messageVision = None
        
        # Convert Lat Lon to UTM.
        self.__lat = 0.0 # Everything is in degrees decimal degrees.
        self.__lon = 0.0
        self.__headingDegrees = 0.0
        self.__headingRadians = 0.0
        self.__turnAngle = 0.0
        self.__turnDirection = self.TURN_STRAIGHT  # 0 is left, 1 is right.
        self.__motorSpeed = self.SPEED_MIN

        self.__compositeX = -1.0 # Composite location coordinates.
        self.__compositeY = -1.0
        
        self.__currentState = self.STATE_STARTUP
        logPrint('Setting current state to: STATE_STARTUP.')
        # We remain in the startup state until we get our first message. 
        # Then we transition to the STATE_SELF_TEST state.
        try:
            self.__dataProcessor = DataProcessor(parent=True, missionPlanner=self)
        except Exception, e:
            logPrint('Failed to create DataProcessor' + str(e), True, LOG_LEVEL_CRITICAL)
            import traceback
            traceback.print_exc()
            self.__currentState = self.STATE_FAILED
            return

        # Get the mission file and parse it.
        loc = MissionLocations(self.__dataProcessor.missionFile)
        self.__locations = loc.locationsUtm
        self.__locationsLatLon = loc.locationsLatLon
        self.__locationName = loc.rteName # Route name.
        
        self.__locationIndex = 1
        logPrint('Current Location: ' + repr(self.__locations[0]))
        self.__setVectorPositionToPrevLocation()
        self.__autoAvoidTimer = TimeoutTimer()
        
        # Show the mission coordinates in a matplotlib scatter plot
        plt.ion()
        self.plotMissionCoordinates()
        return
    
    
    def plotMissionCoordinates(self):
        """Show the mission locations in a scatter plot and annotate them.
        Also show the vector, GPS and composite positions.
        """
        # TODO: show the plot evenly scaled on X and Y including all points
        # plus some margin for printing annotation without needing line wrap.
        # TODO, add arrow for heading of robot.
        plt.clf()
        for loc in self.__locations:
            x = loc[0][0]
            y = loc[0][1]
            plt.text(x, y, ' Type: ' + loc[1] + ', ' + loc[2])
            if loc[1] == self.LOCATION_TYPE_TARGET:
                plt.scatter([x,],[y,], color='r', marker='^')
            else:
                plt.scatter([x,],[y,], color='b', marker='o')
        
        if len(self.__utmGps) != 0:
            x = self.__utmGps[0]
            y = self.__utmGps[1]
            plt.scatter([x,],[y,], color='g', marker='s')
            plt.text(x, y, '  GPS', rotation=-45)
        
        if len(self.__utmVector) != 0:
            x = self.__utmVector[0]
            y = self.__utmVector[1]
            plt.scatter([x,],[y,], color='g', marker='v')
            plt.text(x, y, '  Vector', rotation=-45)
        
        if not self.__compositeX < 0:
            x = self.__compositeX
            y = self.__compositeY
            plt.scatter([x,],[y,], color='g', marker='*')
            plt.text(x, y, '  Composite', rotation=-45)
        
        plt.show()
        plt.draw()
        return
    
    
    def __setVectorPositionToPrevLocation(self):
        """Set the UTM spacial coordinate to the n'th location in the"""
        self.__vectorPositionX = self.__locations[self.__locationIndex-1][0][0]
        self.__vectorPositionY = self.__locations[self.__locationIndex-1][0][1]
        self.__utmVector = (self.__vectorPositionX, self.__vectorPositionY)
        self.__distanceFromLastTarget = 0
        return
    
    
    def __printRangefinderDistances(self):
        message = 'Sonar Left: ' + str(self.__messageSpacial.sonarLeft) \
            + ', Front: ' + str(self.__messageSpacial.sonarFront) \
            + ', Right: ' + str(self.__messageSpacial.sonarRight)
        
        logPrint(message)
        message = 'IR Left: ' + str(self.__messageSpacial.irLeft) \
            + ', IR Right: ' + str(self.__messageSpacial.irRight)
        
        logPrint(message)
        message = 'Bumper Left: ' + str(self.__messageSpacial.bumperLeft) \
            + ', Right: ' + str(self.__messageSpacial.bumperRight)
        
        logPrint(message)
        return
    
    
    def __checkAutoAvoidRequired(self):
        """Check to see if we need to change modes from app control to 
        microcontroller controll."""
        autoAvoidNeeded = False
        
        leftBlocked = self.__messageSpacial.bumperLeft
        rightBlocked = self.__messageSpacial.bumperRight
            
        if self.__messageSpacial.sonarFront < 80 \
            or self.__messageSpacial.sonarLeft < 60 \
            or self.__messageSpacial.sonarRight < 60 \
            or leftBlocked or rightBlocked:
            
            self.__autoAvoidCountdown -= 1
            self.__printRangefinderDistances()
            if self.__autoAvoidCountdown <= 0:
                logPrint('Auto avoidance required:')
                self.__autoAvoidCountdown = self.AUTO_AVOID_DEBOUNCE_COUNT
                autoAvoidNeeded = True
            else:
                # For a bit, simply stop and wait maybe the short distance is 
                # just transient.
                self.__motorSpeed = self.SPEED_STOP
                self.__turnDirection = self.TURN_RIGHT
        
        return autoAvoidNeeded
    
    
    def __checkSlowDownRequired(self):
        """Determine if we need to slow down due to detected objects"""
        obstaclesDetected = False
        
        leftDanger = self.__messageSpacial.irLeft > 80 \
            or self.__messageSpacial.irLeft < 30
        
        rightDanger = self.__messageSpacial.irRight > 80 \
            or self.__messageSpacial.irRight < 30
        
        if self.__messageSpacial.sonarFront < 120 \
            or self.__messageSpacial.sonarLeft < 80 \
            or self.__messageSpacial.sonarRight < 80 \
            or leftDanger or rightDanger:
            
            logPrint('Slow down required:')
            self.__printRangefinderDistances()
            obstaclesDetected = True
        else:
            self.__autoAvoidCountdown = self.AUTO_AVOID_DEBOUNCE_COUNT
        
        return obstaclesDetected
    
    
    def __noObstaclesDetected(self):
        """Determine if we can drive more quickly due to no perceived obstacles"""
        allClear = False
        
        leftClear = self.__messageSpacial.irLeft < 75 \
            and self.__messageSpacial.irLeft > 45 \
            and not self.__messageSpacial.bumperLeft
        
        rightClear = self.__messageSpacial.irRight > 75 \
            and self.__messageSpacial.irRight < 45 \
            and not self.__messageSpacial.bumperRight
        
        if self.__messageSpacial.sonarFront > 150 \
            and self.__messageSpacial.sonarLeft > 100 \
            and self.__messageSpacial.sonarRight > 100 \
            and leftClear and rightClear:
            
            logPrint('No obstacles detected:')
            self.__printRangefinderDistances()
            self.__autoAvoidCountdown = self.AUTO_AVOID_DEBOUNCE_COUNT
            allClear = True
        
        return allClear
    
    
    def __getCurrentLocationType(self):
        """Get the current location type based on the location index. The 
        locations list is a list of tuples: 
            tuple: UTM location, length 2, meters easting, meters northing
            string: location type
            string: location handle
        """
        if self.__locationIndex < len(self.__locations):
            return self.__locations[self.__locationIndex][1]
        else:
            return self.__locations[self.__locationIndex-1][1]
    
    
    def __selfTest(self):
        """Perform a subsystem self-test.
        Start a counter and if a limit is exceeded, then go into
        the failed state."""
        # Ensure that regardless of the initial encoder value, the delta is not
        # large (if in a rare case every subsystem was initialized for first 
        # spacial message).
        self.__prevEncoderCountsLeft = self.__encoderCountsLeft
        self.__prevEncoderCountsRight = self.__encoderCountsRight
        self.__failedStateCounter += 1
        if self.__failedStateCounter > self.FAILED_STATE_COUNT_LIMIT:
            self.__currentState = self.STATE_FAILED
            logger.error('Failed state counter exceeded failed state count limit')
            logPrint('Setting current state to: STATE_FAILED.')
            print 'Failed state counter exceeded failed state count limit'
            logPrint('Inertial System: ' + repr(self.__messageInertial))
            logPrint('GPS System: ' + repr(self.__messageGps))
            logPrint('Vision System: ' + repr(self.__messageVision))
            self.shutdown()
        if self.__messageInertial is not None \
            and self.__messageVision is not None:
            
            if self.__messageGps is None:
                # It's not so great that we don't have GPS, but under some 
                # conditions, we should operate without it.
                self.__currentState = self.STATE_IMPAIRED
                logPrint('Setting current state to: STATE_IMPAIRED.')
            else:
                self.__currentState = self.STATE_NOMINAL
                logPrint('Setting current state to: STATE_NOMINAL.')
            # Set the operating mode.
            self.setRunMode(self.MODE_WAYPOINT_SEARCH)
            logPrint('Setting current mode to: MODE_WAYPOINT_SEARCH.')
        return
    
    
    def __checkShutdown(self):
        if self.__shutdownRequested:
            self.__shutdownCountdown -= 1
            self.setRunMode(self.MODE_HALT)
            if self.__shutdownCountdown <= 0:
                self.__currentState = self.STATE_SHUTDOWN
                self.__dataProcessor.shutdown()
        return
    
    
    def __normalOperation(self):
        """As we would expect, we'll spend most of our time here. 
        Control actuators, etc..
        """
        self.__checkShutdown()
        # The following three functions must be called in order:
        # __computeSpacialPosition()
        # __computeLocationDistance()
        # __computeTurnDirection()
        
        self.__computeSpacialPosition()
        
        if self.currentMode == self.MODE_HALT:
            if self.__radioMode != self.RADIO_MODE_MOTION_ENABLED_REMOTE:
                self.setMotorSpeed(self.SPEED_STOP, self.SPEED_STOP)
            return
        
        # If we are at the last location, then halt.
        if self.__locationIndex >= len(self.__locations):
            # Halt the robot
            logPrint('Exhausted locations to traverse, stopping.')
            self.setRunMode(self.MODE_HALT)
            self.setMotorSpeed(self.SPEED_STOP, self.SPEED_STOP)
            self.__motorSpeed = self.SPEED_STOP
            return
        
        self.__computeLocationDistance()
        self.__computeTurnDirection()
        
        if self.__noObstaclesDetected():
            self.__motorSpeed = self.SPEED_ULTRA
        elif self.__checkSlowDownRequired() or self.__crosstrackError > 45.0:
            # Slow dowm when cross track error is big.
            self.__motorSpeed = self.SPEED_MIN
        elif self.__crosstrackError > 25.0:
            self.__motorSpeed = self.SPEED_MED
        else:
            self.__motorSpeed = self.SPEED_MAX
        
        # If we are within a threshold distance from an intermediate location
        # then increment to the next location.
        currentLocationType = self.__getCurrentLocationType()
        
        if currentLocationType == self.LOCATION_TYPE_WAYPOINT \
            and self.__distanceToLocation <= self.THRESHOLD_WAYPOINT_DISTANCE:
            self.__locationIndex += 1
            logPrint('Reached intermediate waypoint, index: ' \
                     + str(self.__locationIndex))
            
        elif currentLocationType == self.LOCATION_TYPE_TARGET \
            and self.__distanceToLocation <= self.THRESHOLD_TARGET_DISTANCE:
            
            if self.__targetShapeVisible or self.__targetShapeLocked:
                
                if self.currentMode != self.MODE_TARGET_TRACKING:
                    # Stop for a bit. Take a picture.
                    self.__motorSpeed = self.SPEED_STOP
                    self.saveImages()
                else:
                    self.__motorSpeed = self.SPEED_MED
                
                self.setRunMode(self.MODE_TARGET_TRACKING)
                self.__moveTowardTarget()
                if self.__distanceToTarget > 0 and self.__distanceToTarget < 60:
                    # 30 CM as ranged by the center sonar rangefinder
                    # TODO: use a constant instead of numeric literal 30.
                    logger.info('Ranging target: approaching target at slow speed.')
                    self.__motorSpeed = self.SPEED_MIN
                
                # Change this criteria given bumper switches.
                if self.__messageSpacial.bumperLeft or self.__messageSpacial.bumperRight:
                    logPrint('Contacted target, incrementing location index: ' \
                             + str(self.__locationIndex))
                    
                    self.saveImages()
                    self.__locationIndex += 1
                    self.__targetSearchDistance = 0
                    # Set the target shape locked member to false after index increment..
                    self.__targetShapeLocked = False
                    
                    # Reset our vector location to the current one in the list. 
                    # We know exactly where we are because we contacted the target.
                    self.__setVectorPositionToPrevLocation()
                    self.setRunMode(self.MODE_WAYPOINT_SEARCH)
                    self.__currentSearchPattern = self.SEARCH_PATTERN_NA
                
            else:
                self.setRunMode(self.MODE_TARGET_SEARCH)
                self.__searchForTarget()
        
        if (self.currentMode == self.MODE_WAYPOINT_SEARCH \
            or self.currentMode == self.MODE_TARGET_SEARCH) \
            and self.__checkAutoAvoidRequired():
            
            self.setRunMode(self.MODE_AUTO_AVOID)
            self.__autoAvoidTimer.resetTimeout(2) # 4.0 ~ 10.0 seconds
        elif self.__autoAvoidTimer.checkTimeout() < 0 \
            and self.currentMode == self.MODE_AUTO_AVOID:
            
            # Revert to mission planner control after a timeout.
            self.setRunMode(self.MODE_WAYPOINT_SEARCH)
        
        if self.__spacialUpdateCounter % self.SPEED_UPDATE_INTERVAL == 0:
            speedLeft = 0
            speedRight = 0
            
            if self.__turnDirection == self.TURN_LEFT:
                speedLeft = int(self.__motorSpeed - self.__turnAngle)
                speedRight = int(self.__motorSpeed)
            
            elif self.__turnDirection == self.TURN_RIGHT:
                speedLeft = int(self.__motorSpeed)
                speedRight = int(self.__motorSpeed - self.__turnAngle)
            elif self.__turnDirection == self.TURN_STRAIGHT \
                or self.__motorSpeed == self.SPEED_STOP:
                
                speedLeft = int(self.__motorSpeed)
                speedRight = int(self.__motorSpeed)
            
            #print speedLeft, speedRight
            
            # Set the speed of the right and left motor's
            self.setMotorSpeed(speedLeft, speedRight)
        
        return
    
    
    def updateSpacial(self, message):
        """All robot control is driven by spacial update message events.
        """
        self.__spacialUpdateCounter += 1
        self.__messageSpacial = message
        self.__prevEncoderCountsLeft = self.__encoderCountsLeft
        self.__prevEncoderCountsRight = self.__encoderCountsRight
        self.__radioMode = self.__messageSpacial.radioMode
        self.__encoderCountsLeft = self.__messageSpacial.encoderCountsLeft
        self.__encoderCountsRight = self.__messageSpacial.encoderCountsRight
        # We could compute deltas here, but the first one may not be accurate
        # if the encoder counts don't start at zero.
        
        # Update the system state machine.
        if self.__currentState == self.STATE_STARTUP:
            self.__currentState = self.STATE_SELF_TEST
            logPrint('Setting current state to: STATE_SELF_TEST.')
        elif self.__currentState == self.STATE_SELF_TEST:
            self.__selfTest()
        elif self.__currentState == self.STATE_NOMINAL \
                or self.__currentState == self.STATE_IMPAIRED:
            self.__normalOperation()
        return
    
    
    def updateInertial(self, message, yawRadians, yawDegrees):
        """ Mainly used to set the heading so that we can compute cross-track 
        error, and turning direction.
        """
        self.__messageInertial = message
        self.__headingDegrees = yawDegrees
        self.__headingList.append(yawRadians)
        
        #print yawDegrees
        if len(self.__headingList) > 5:
            self.__headingList.remove(self.__headingList[0])
        return
    
    
    def updateGps(self, message, lat, lon):
        """ Format: dd.dd...
        
        Update the GPS position, if we are within sight of a target, change 
        mode and begin searching for it.
        """
        self.__messageGps = message
        # Decompose the message, extract relevant fields, mainly Lat/Lon.
        self.__lat = lat
        self.__lon = lon
        self.__computeGpsPosition()
        return
    
    
    def updateVision(self, message):
        """Update the vision information. Can see target...
        """
        self.__messageVision = message
        self.__targetShapeVisible = message.targetShapeVisible
        self.__targetShapeX = message.targetShapeX
        self.__targetShapeY = message.targetShapeY
        self.__targetScale = message.targetScale
        if self.__targetShapeVisible \
            and self.__targetScale > self.TARGET_SCALE_THRESH \
            and self.__getCurrentLocationType() == self.LOCATION_TYPE_TARGET \
            and self.__distanceToLocation <= self.THRESHOLD_TARGET_DISTANCE:
            # Lock in the fact that we are seeing the cone.
            self.__targetShapeLocked = True
        
        if self.__targetShapeLocked and not self.__targetShapeVisible:
            # If we are locked on the target, just use the center of mass of 
            # the target color, unless the target shape is actually visible,
            # then use the target shape location, since that is more accurate.
            self.__targetShapeX = message.targetColorX
            self.__targetShapeY = message.targetColorY
        return
    
    
    def shutdown(self):
        """Unload the mission planner,
        """
        if self.__shutdownRequested:
            # We called this twice, do it right now.
            self.__currentState = self.STATE_SHUTDOWN
            self.setRunMode(self.MODE_HALT)
            self.setMotorSpeed(self.SPEED_STOP, self.SPEED_STOP)
            self.__dataProcessor.shutdown()
        self.__shutdownRequested = True
        return
    
    
    def takePicture(self):
        print 'Mission Planner: send picture taking message.'
        self.__dataProcessor.sendMessage('c:pic')
        return
    
    
    def showCamera(self):
        print 'Mission Planner: send show camera message.'
        self.__dataProcessor.sendMessage('c:show')
        return
    
    
    def saveImages(self):
        print 'Mission Planner: send save images message to camera.'
        self.__dataProcessor.sendMessage('c:save')
        return
    
    
    def setMotorSpeed(self, speedLeft, speedRight):
        """Speed: signed integer, 0 to 255."""
        self.__dataProcessor.sendMessage('s:l:' + str(speedLeft) + ';')
        self.__dataProcessor.sendMessage('s:r:' + str(speedRight) + ';')
        return
    
    
    def setRunMode(self, mode):
        """Set system and spacial controller run modes.
        """
        if mode == self.__currentMode:
            return
        self.__currentMode = mode
        logPrint('Setting system controller mode: ' + self.MODE_NAMES[mode])
        # Set the run mode of the spacial controller
        if mode == self.MODE_AUTO_AVOID:
            self.__setRunModeSpacial(self.SPACIAL_MODE_AUTO_AVOID)
        elif mode == self.MODE_REMOTE_CONTROL:
            self.__setRunModeSpacial(self.SPACIAL_MODE_REMOTE_CONTROL)
        elif mode == self.MODE_HALT:
            self.__setRunModeSpacial(self.SPACIAL_MODE_SYS_CONTROL)
        else:
            self.__setRunModeSpacial(self.SPACIAL_MODE_SYS_CONTROL)
        return
    
    
    def __setRunModeSpacial(self, mode):
        """Set spacial run mode, send message to spacial controller.
        """
        if self.__currentSpacialMode != mode:
            logPrint('Setting spacial controller mode: ' + self.SPACIAL_MODE_NAMES[mode])
            self.__dataProcessor.sendMessage('s:m:' + str(mode) + ';')
            self.__currentSpacialMode = mode
        return
    
    
    def showMap(self, showMap=False):
        """""" 
        try:
            self.__staticMap.getMapImage((self.__lat, self.__lon)
                                         , self.__locationsLatLon
                                         , self.__locationName
                                         , showMap)
        except Exception, e:
            print e # and log it?
            pass
        return
    
    
    def __computeGpsPosition(self):
        # Convert lat lon to UTM
        if self.__lat != 0.0:
            self.__utmGps = utm.from_latlon(self.__lat, self.__lon)
        return
    
    
    def __computeSpacialPosition(self):
        """Sum the next vector from the spacial system.
        Compute the delta counts and average the two, combine this with 
        the compass direction to produce a vector."""
        deltaL = self.__encoderCountsLeft - self.__prevEncoderCountsLeft
        deltaR = self.__encoderCountsRight - self.__prevEncoderCountsRight
        deltaCounts = (deltaL + deltaR) / 2.0 # Only works for skid steering.
        
        # Average the the headings in the heading list. Then clear it.
        # If the heading list is empty, then simply use the previous heading.
        headingSum = 0
        averageHeading = self.__prevAverageHeading
        for heading in self.__headingList:
            headingSum += heading
        if len(self.__headingList) > 0:
            averageHeading = headingSum / float(len(self.__headingList))
        # Average the current and previous heading if we only got a single
        # heading value.
        if len(self.__headingList) == 1 and self.__prevAverageHeading != -1.0:
            averageHeading = (averageHeading + self.__prevAverageHeading) / 2.0
        self.__averageHeading = averageHeading
        
        # If we did not move, then we don't need to update this information.
        if deltaCounts == 0:
            return
        
        # Convert the delta counts to meters.
        deltaMeters = deltaCounts / self.__dataProcessor.encoderCountsPerMeter
        self.__loopDisplacement = deltaMeters # How far we moved this iteration.
        self.__distanceFromLastTarget += deltaMeters
        self.__vectorDisplacement += deltaMeters
        #print 'Delta Meters: ', deltaMeters, ', Average Heading', averageHeading
        # Add the X and Y values to the UTM vector values.
        self.__vectorPositionX += deltaMeters * math.cos(averageHeading)
        self.__vectorPositionY += deltaMeters * math.sin(averageHeading)
        
        # Assign a new updated tuple value to the spacial UTM variable.
        self.__utmVector = (self.__vectorPositionX, self.__vectorPositionY)
        self.__headingList = []
        self.__prevAverageHeading = self.__averageHeading
        return
    
    
    def __computeInertialPosition(self):
        # Currently unused
        return
    
    
    def __computeCompositePosition(self, gX, gY, vX, vY, gpsValid):
        """Combine the position estimations:
        For now, simply average the GPS and spacial positions. Weight the
        GPS position by the number of satelites used (above some minimum)
        and weight the spacial position by the inverse of the distance
        travelled (since the accuracy of the vector position goes down as
        distance from a last known location increases."""
        gpsWeight = 0.0
        if gpsValid:
            # Weigh the GPS position based on the number of satelites in view.
            gpsWeight = float(self.__messageGps.fields.sat)
        # Weigh the vector position by the nearness to the last known target.
        vectorWeight = 100.0 / (self.__distanceFromLastTarget + 1)
        cX = (gX * gpsWeight + vX * vectorWeight) / (gpsWeight + vectorWeight)
        cY = (gY * gpsWeight + vY * vectorWeight) / (gpsWeight + vectorWeight)
        self.__positionConfidence = gpsWeight + vectorWeight
        self.__compositeX = cX
        self.__compositeY = cY
        return (cX, cY)
    
    
    def __computeLocationDistance(self):
        """Calculate the distance between where we are and where we want to go.
        """
        gpsValid = False
        gX = 0.0
        gY = 0.0
        if len(self.__utmGps) != 0:
            gpsValid = True
            gX = self.__utmGps[0]
            gY = self.__utmGps[1]
        
        vX = self.__vectorPositionX
        vY = self.__vectorPositionY

        currX, currY = self.__computeCompositePosition(gX, gY, vX, vY, gpsValid)
        
        goalX = self.__locations[self.__locationIndex][0][0]
        goalY = self.__locations[self.__locationIndex][0][1]
        
        self.__distanceToLocation = math.sqrt(math.pow(goalX - currX, 2) +
                                              math.pow(goalY - currY, 2))
        
        return self.__distanceToLocation
    
    
    def __computeTurnDirection(self):
        """Given our current location and heading, compute the direction we 
        must turn to go towards the next location.
        """
        goalX = self.__locations[self.__locationIndex][0][0]
        goalY = self.__locations[self.__locationIndex][0][1]
        
        heading = self.__headingDegrees
        # Angle between east and where we are and where we want to be.
        angle = math.atan2(goalY - self.__compositeY, goalX - self.__compositeX)

        angle *= 180.0 / math.pi # Convert radians to degrees.
        
        if angle < 0.0:
            angle += 360.0
        # Now figure out which is the least angle between the direction we are 
        # heading and the direction we want to face.
        
        # And determine whether we turn right or left.
        turnAngle = abs(heading - angle)
        if turnAngle > 180.0:
            turnAngle = 360.0 - abs(heading - angle)
        
        self.__turnAngle = turnAngle * self.TURN_RATIO
        self.__crosstrackError = turnAngle
        if self.__crosstrackError < 2.0:
            self.__turnAngle = 0
        
        if angle - heading < 0 and abs(angle - heading) >= 180:
            self.__turnDirection = self.TURN_LEFT
        elif angle - heading > 0 and abs(angle - heading) >= 180:
            self.__turnDirection = self.TURN_RIGHT
        elif angle - heading < 0 and abs(angle - heading) < 180:
            self.__turnDirection = self.TURN_RIGHT
        elif angle - heading > 0 and abs(angle - heading) < 180:
            self.__turnDirection = self.TURN_LEFT
        else:
            self.__turnDirection = self.TURN_STRAIGHT
        return turnAngle
    
    
    def __searchForTarget(self):
        """Here we execute target search patterns, but for now, do nothing.
        The targetShapeVisible flag is already set in the updateVision function."""
        
        DISTANCE_LIMIT_DIVISION = self.TARGET_SEARCH_DISTANCE_LIMIT / 3.0
        self.__targetSearchDistance += self.__loopDisplacement
        if self.__targetSearchDistance < DISTANCE_LIMIT_DIVISION:
            self.__currentSearchPattern = self.SEARCH_PATTERN_1
            # Keep going toward where we think the cone is. The turn direction 
            # and magnitude are set previously in compute turn direction
        elif self.__targetSearchDistance < DISTANCE_LIMIT_DIVISION * 2:
            self.__currentSearchPattern = self.SEARCH_PATTERN_2
            self.__turnDirection = self.TURN_LEFT
            self.__turnAngle = 20
        elif self.__targetSearchDistance < DISTANCE_LIMIT_DIVISION * 3:
            self.__currentSearchPattern = self.SEARCH_PATTERN_3
            self.__turnDirection = self.TURN_RIGHT
            self.__turnAngle = 20
        else:
            self.__currentSearchPattern = self.SEARCH_PATTERN_NA
            logPrint('Giving up on target, incrementing location index.')
            self.__locationIndex += 1
            self.__targetSearchDistance = 0
            self.__targetShapeLocked = False
            self.setRunMode(self.MODE_WAYPOINT_SEARCH)
        return
    
    
    def __moveTowardTarget(self):
        """Here we determine which direction to turn to drive toward the cone.
        If we are close enough, go really slow, we also decide if have 
        made contact with the target here, basically, when there is a 
        minimum distance to the target."""
        distanceToTarget = self.__messageSpacial.sonarFront
        # basically, we center the target in the image.
        if self.__targetShapeX < self.CAMERA_HORIZONTAL_CENTER:
            self.__turnDirection = self.TURN_LEFT
        elif self.__targetShapeX > self.CAMERA_HORIZONTAL_CENTER:
            self.__turnDirection = self.TURN_RIGHT
        else:
            self.__turnDirection = self.TURN_STRAIGHT
            
        pixelError = 6000.0 / distanceToTarget
        self.__turnAngle = abs(self.__targetShapeX - self.CAMERA_HORIZONTAL_CENTER)
        self.__turnAngle *= self.TURN_RATIO_VISION
        
        if self.__turnAngle < pixelError:
            self.__distanceToTarget = distanceToTarget
        else:
            # Target is not being ranged. Set to -1 invalid.
            self.__distanceToTarget = -1
        
        message = 'Moving toward target x: ' + str(self.__targetShapeX) \
            + ', Scale: ' + str(self.__targetScale) \
            + ', Locked: ' + str(self.__targetShapeLocked) \
            + ', Distance: ' + str(self.__distanceToTarget) \
            + ', Direction: ' + self.TURN_DIRECTION_NAMES[self.__turnDirection] \
            + ', Angle: ' + str(self.__turnAngle)
        
        logPrint(message)
        return
    
    
    def showDebug(self):
        pass
        # TODO: make a more advanced debug user interface.
        #foo = Ui_MainWindow()
        self.plotMissionCoordinates()
        return
    
    
    def printDebug(self):
        #print '\x1b[2J\x1b[H' # This clears the console apparently
        print '==============================================================='
        if self.__messageInertial is not None:
            print '[ Inertial System Data: ]'
            print '*************************'
            print '<Magnetometer:>'
            print '---------------'
            print 'Heading:', self.__messageInertial.heading
            print 'Pitch:  ', self.__messageInertial.pitch
            print 'Roll:   ', self.__messageInertial.roll
            """
            print 'Speed X:', self.__messageInertial.speedX # First Integral
            print 'Speed Y:', self.__messageInertial.speedY
            print 'Speed Z:', self.__messageInertial.speedZ
            print 'Distance X:', self.__messageInertial.distanceX # Second Integral
            print 'Distance Y:', self.__messageInertial.distanceY
            print 'Distance Z:', self.__messageInertial.distanceZ
            print '<Rate Gyroscope:>'
            print '-----------------'
            print 'Gyro X: ', self.__messageInertial.gyroX
            print 'Gyro Y: ', self.__messageInertial.gyroY
            print 'Gyro Z: ', self.__messageInertial.gyroZ
            """
            print '<Inertial System Status:>'
            print '-------------------------'
            print 'In Motion:      ', self.__messageInertial.inMotion
            #print 'System Cal:     ', self.__messageInertial.calSystem
            print 'Gyro Cal:       ', self.__messageInertial.calGyro
            #print 'Accel Cal:      ', self.__messageInertial.calAccel
            print 'Compass Cal:    ', self.__messageInertial.calMag
            print 'Temperature C:  ', self.__messageInertial.temperature
        
        if self.__messageSpacial is not None:
            print '[ Spacial System Data: ]'
            print '************************'
            print '<Sonar Distances (cm):>'
            print '-----------------------'
            print 'Sonar Left:  ', self.__messageSpacial.sonarLeft
            print 'Sonar Front: ', self.__messageSpacial.sonarFront
            print 'Sonar Right: ', self.__messageSpacial.sonarRight
            print 'Sonar Back:  ', self.__messageSpacial.sonarBack
            print '<Infrared Distances (cm):>'
            print '--------------------------'
            print 'IR Left:  ', self.__messageSpacial.irLeft
            print 'IR Right: ', self.__messageSpacial.irRight
            print 'IR Back:  ', self.__messageSpacial.irBack
            print '<Bumper Switch States:>'
            print '-----------------------'
            print 'Bumper Left:  ', self.__messageSpacial.bumperLeft
            print 'Bumper Right: ', self.__messageSpacial.bumperRight
            print '<Encoder Counts:>'
            print '-----------------'
            print 'Encoder Counts Left:  ', self.__messageSpacial.encoderCountsLeft
            print 'Encoder Counts Right: ', self.__messageSpacial.encoderCountsRight
            print 'Motor Status Left:  ', self.__messageSpacial.motorStatusLeft
            print 'Motor Status Right: ', self.__messageSpacial.motorStatusRight
            print '<Radio Status:>'
            print '---------------'
            print 'Radio Mode:', self.RADIO_MODE_NAMES[self.__radioMode]
            print 'Radio Enabled: ', self.__messageSpacial.radioEnabled
        
        print '[ Camera Tracking Data: ]'
        print '*************************'
        print 'Target Visible: ', self.__targetShapeVisible
        print 'Target Shape Locked: ', self.__targetShapeLocked
        print 'Target X: ', self.__targetShapeX
        print 'Target Y: ', self.__targetShapeY
        print 'Target Size: ', self.__targetScale
        print 'Distance to Target: ', self.__distanceToTarget
        
        if self.messageGps is not None and self.messageGps.fields is not None:
            print '[ GPS Position Data: ]'
            print '**********************'
            print 'Latitude:  ', self.__lat
            print 'Longitude: ', self.__lon
            print 'UTM:  ', repr(self.__utmGps)
            print 'Satelite Fix:  ', self.messageGps.fields.fix
            print 'Satelite Count:', self.messageGps.fields.sat
        
        print '[ Vector Position Data: ]'
        print '*************************'
        print 'Vector Position X: ', self.__vectorPositionX
        print 'Vector Position Y: ', self.__vectorPositionY
        print 'Vector Displacement: ', self.__vectorDisplacement
        
        print '[ Composite Values: ]'
        print '*********************'
        print 'Turn Direction: ', self.TURN_DIRECTION_NAMES[self.__turnDirection]
        print 'Crosstrack Error: ', self.__crosstrackError
        print 'Turn Magnitude: ', self.__turnAngle
        print 'Speed:  ', self.__motorSpeed
        print 'Distance to waypoint:', self.__distanceToLocation
        print 'Composite X:', self.__compositeX
        print 'Composite Y:', self.__compositeY
        
        print '[ System Status: ]'
        print '******************'
        print 'System State: ', self.currentStateName
        print 'System Mode:  ', self.currentModeName
        print 'Spacial Mode: ', self.currentSpacialModeName
        print 'Current Waypoint Index: ', self.__locationIndex
        if self.__locationIndex < len(self.__locations):
            print 'Current Waypoint: ', self.__locations[self.__locationIndex]
        else:
            print 'Current Waypoint: ', self.__locations[self.__locationIndex-1]
        print 'Speed:  ', self.__motorSpeed
        print 'Search Pattern:  ', self.SEARCH_PATTERN_NAMES[self.__currentSearchPattern]
        print 'Search Distance:  ', self.__targetSearchDistance
        
        # TODO: print meters traveled GPS and Vector.
        return
    
    
    # Properties: 
    @property
    def lat(self):
        return self.__lat
    
    @property
    def lon(self):
        return self.__lon
    
    @property
    def currentState(self):
        return self.__currentState
    
    @property
    def currentStateName(self):
        return self.STATE_NAMES[self.__currentState]
    
    @property
    def currentMode(self):
        return self.__currentMode
    
    @property
    def currentModeName(self):
        return self.MODE_NAMES[self.__currentMode]
    
    @property
    def currentSpacialModeName(self):
        return self.SPACIAL_MODE_NAMES[self.__currentSpacialMode]
    
    @property
    def messageSpacial(self):
        return self.__messageSpacial
    
    @property
    def messageInertial(self):
        return self.__messageInertial
    
    @property
    def messageGps(self):
        return self.__messageGps
    
    @property
    def messageVision(self):
        return self.__messageVision
    
    

###############################################################################
# Basic commands accessable from the command line
###############################################################################

def start():
    # Instantiate the data processor as the parent thread.
    mp = MissionPlanner()
    return mp


if __name__ == "__main__":
    # Instantiate the data processor as the parent.
    # TODO: parse argv for configuration file name and pass it to.
    mp = start()


def stop():
    mp.shutdown()
    return


# Show Map:
def sm():
    mp.showMap(True)
    #t = Timer(1.0, show)
    #t.start() 
    return


def pic():
    # Take a picture and display it.
    #dp.sendMessage('c:p:' + int(show) + ';')
    mp.saveImages()
    return


def setSpeed(speedLeft=0, speedRight=0):
    # Set the left motor speed:
    mp.setMotorSpeed(speedLeft, speedRight)
    return


# Normal operating modes, may also be used in the development state:
# Modes are externally visible behavior modes. 
MODE_UNKNOWN = -1
MODE_WAYPOINT_SEARCH = 0 # Use vector navigation and GPS, move to waypoint. 
MODE_TARGET_SEARCH = 1   # Looking for a target object (visual search)
MODE_TARGET_TRACKING = 2 # Target is visually acquired, track it.
MODE_CALIBRATION = 3     # Calibrate inertial measurement subsystem.
MODE_AUTO_AVOID = 4      # An obstacle is detected, allow subsystem to control
MODE_REMOTE_CONTROL = 5  # Control the robot with a remote control
MODE_HALT = 6            # Stop all motion

def mode(mode):
    mp.setRunMode(mode)
    return

def speed(speed):
    setSpeed(speed, speed)
    return

def turn(speed, direction):
    speedLeft = speed + direction
    speedRight = speed - direction
    setSpeed(speedLeft, speedRight)
    return

# Show Camera
def sc():
    mp.showCamera()
    return

# Show Debug
def sd():
    """Show the debug UI, not implemented"""
    mp.showDebug()
    return

# TODO: enable conditional printing of debug data.
printDebugInertial = True
printDebugSpacial = True

# Print Debug:
def pd(period=0):
    # Period in seconds.
    while True:
        mp.printDebug()
        if period == 0: break
        time.sleep(period) # Sleep in seconds.
    return


