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
import data_processor
from data_processor import DataProcessor
from static_maps import StaticMap

from debug_view import Ui_MainWindow

logger = data_processor.logger

def logPrint(message, printMessage=True):
    logger.info(message)
    if printMessage:
        print message
    return


class TimeoutTimer(object):
    
    def __init__(self, duration = 0):
        """Initialize the duration field. Duration is in tens of seconds, 
        floating point"""
        self.resetTimeout(duration)
    
        
    def checkTimeout(self):
        """Return the number of tens of seconds until timeout.
        If the duration has not been elapsed, return a positive number, 
        otherwise the return value is negative."""
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
    STATE_UNKNOWN     = -1
    STATE_STARTUP     = 0 # State used when bringing up subsystems.
    STATE_SELF_TEST   = 1 # State used when testing subsystems.
    STATE_NORMAL_OPERATION = 2 # All self tests passed.
    STATE_FAILED      = 3 # Used when one or more critical subsystems fail.
    STATE_IMPAIRED    = 4 # Used when a non-critical subsystem is down.
    STATE_DEVELOPMENT = 5 # Dunno, what was this for?
    STATE_SHUTDOWN    = 7 # 
    
    STATE_NAMES = { STATE_UNKNOWN     : 'STATE_UNKNOWN'
                  , STATE_STARTUP     : 'STATE_STARTUP'
                  , STATE_SELF_TEST   : 'STATE_SELF_TEST'
                  , STATE_NORMAL_OPERATION : 'STATE_NORMAL_OPERATION'
                  , STATE_FAILED      : 'STATE_FAILED'
                  , STATE_IMPAIRED    : 'STATE_IMPAIRED'
                  , STATE_DEVELOPMENT : 'STATE_DEVELOPMENT'
                  , STATE_SHUTDOWN    : 'STATE_SHUTDOWN' }
    
    __currentState = STATE_UNKNOWN
    logPrint('Setting current state to: STATE_UNKNOWN.')
    
    # Normal operating modes, may also be used in the development state:
    # Modes are externally visible behavior modes. 
    MODE_UNKNOWN = -1
    MODE_WAYPOINT_SEARCH = 0 # Use vector navigation and GPS, move to waypoint. 
    MODE_TARGET_SEARCH = 1 # Looking for a target object (visual search)
    MODE_TARGET_TRACKING = 2 # Target is visually acquired, track it.
    MODE_CALIBRATION = 3 # Calibrate inertial measurement subsystem.
    MODE_AUTO_AVOID = 4 # An obstacle is detected, allow subsystem to control
    MODE_HALT = 5
    
    MODE_NAMES = { MODE_UNKNOWN         : 'MODE_UNKNOWN'
                 , MODE_WAYPOINT_SEARCH : 'MODE_WAYPOINT_SEARCH'
                 , MODE_TARGET_SEARCH   : 'MODE_TARGET_SEARCH'
                 , MODE_TARGET_TRACKING : 'MODE_TARGET_TRACKING'
                 , MODE_CALIBRATION     : 'MODE_CALIBRATION'
                 , MODE_AUTO_AVOID      : 'MODE_AUTO_AVOID'
                 , MODE_HALT            : 'MODE_HALT' }
    
    __currentMode = MODE_UNKNOWN
    logPrint('Setting current mode to: MODE_UNKNOWN.')
    
    # Spacial controllers modes of operation, see 
    SPACIAL_MODE_AUTO_AVOID = 0     # Use sensors to automatically avoid objects.
    SPACIAL_MODE_EXT_CONTROL = 1    # Rely on external software control commands.
    SPACIAL_MODE_REMOTE_CONTROL = 2 # Rely on remote (radio) control for movement.
    
    SPACIAL_MODE_NAMES = { SPACIAL_MODE_AUTO_AVOID     : 'SPACIAL_MODE_AUTO_AVOID'
                         , SPACIAL_MODE_EXT_CONTROL    : 'SPACIAL_MODE_EXT_CONTROL'
                         , SPACIAL_MODE_REMOTE_CONTROL : 'SPACIAL_MODE_REMOTE_CONTROL'
                         }
    
    __currentSpacialMode = SPACIAL_MODE_EXT_CONTROL
    
    THRESHOLD_TARGET_DISTANCE = 15 # Meters to target before we look for it...
    THRESHOLD_LOCATION_DISTANCE = 10 # Meters to location before we go to the next one.
    
    TURN_LEFT = 0
    TURN_RIGHT = 1
    TURN_STRAIGHT = 2
    
    TURN_RATIO = 10.0 / 180.0
    
    # TODO: get the horizontal resolution from the camera implementation.
    TURN_RATIO_VISION = 10.0 / (640 / 2.0)  
    
    SPEED_MAX = 10.0
    SPEED_MED = 7.0
    SPEED_MIN = 4.0
    SPEED_UPDATE_INTERVAL = 10
    
    LOCATION_TYPE_START = 'start'
    LOCATION_TYPE_WAYPOINT = 'waypoint'
    LOCATION_TYPE_TARGET = 'target'
    LOCATION_TYPE_FINISH = 'finish' # Don't use this one.
    
    # We always start at the start location :).
    __curentLocationType = LOCATION_TYPE_START
    
    FAILED_STATE_COUNT_LIMIT = 100
    
    __staticMap = StaticMap()
    
    def __init__(self):
        """ Constructor."""
        # The target location list is a list of locations the robot must go.
        # Start, location 1, 2, 3, etc..
        # Format: name: (lat, lon), type
        # The location type specifies what we must do once we get to the location.
        # The location types are: start, waypoint, target, and finish.
        # The start location type is the starting location.
        # The waypoint location type is an intermediate location, used to 
        # direct the robot away from hazards, etc..
        
        # The distance to the next/current location in the location list.
        self.__distanceToLocation = sys.maxint
        # A boolean flag, True when the vision system has spotted the target.
        self.__targetVisible = False
        # The distance to the target, measured by the range sensor
        self.__distanceToTarget = sys.maxint
        self.__failedStateCounter = 0
        self.__spacialUpdateCounter = 0
        
        self.__utmGps = ()
        self.__utmVector = ()
        
        self.__initialEncoderCountsLeft = 0
        self.__initialEncoderCountsRight = 0
        self.__encoderCountsLeft = 0
        self.__encoderCountsRight = 0
        self.__prevEncoderCountsLeft = 0
        self.__prevEncoderCountsRight = 0
        
        self.__averageHeading = 0.0
        self.__prevAverageHeading = 0.0
        self.__headingList = []
        
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
        self.__turnDirection = self.TURN_LEFT  # 0 is left, 1 is right.
        self.__motorSpeed = self.SPEED_MIN

        self.__currentState = self.STATE_STARTUP
        logPrint('Setting current state to: STATE_STARTUP.')
        # We remain in the startup state until we get our first message. 
        # Then we transition to the STATE_SELF_TEST state.
        self.__dataProcessor = DataProcessor(parent=True, missionPlanner=self)

        # Get the mission file and parse it.
        loc = MissionLocations(self.__dataProcessor.missionFile)
        self.__locations = loc.locationsUtm
        self.__locationsLatLon = loc.locationsLatLon
        self.__locationName = loc.rteName # Route name.
        
        self.__locationIndex = 1
        logPrint('Current Goal Location: ' + repr(self.__locations[0]))
        
        self.__autoAvoidTimer = TimeoutTimer()
        
        return
    
    def __checkAutoAvoidRequired(self):
        """Check to see if we need to change modes from app control to 
        microcontroller controll."""
        # TODO: set constant thresholds here.
        return self.__messageSpacial.sonarFront < 40 \
            or self.__messageSpacial.sonarLeft < 20 \
            or self.__messageSpacial.sonarRight < 20
    
    
    def __selfTest(self):
        """Perform a subsystem self-test."""
        # Start a counter and if a limit is exceeded, then go into
        # the failed state.
        self.__failedStateCounter += 1
        if self.__failedStateCounter > self.FAILED_STATE_COUNT_LIMIT:
            self.__currentState = self.STATE_FAILED
            logger.error('Failed state counter exceeded failed state count limit')
            logPrint('Setting current state to: STATE_FAILED.')
            print 'Failed state counter exceeded failed state count limit'
            logPrint('Inertial System: ' + repr(self.__messageInertial))
            logPrint('GPS System: ' + repr(self.__messageGps))
            logPrint('Vision System: ' + repr(self.__messageVision))
            stop()
        if self.__messageInertial is not None \
            and self.__messageVision is not None:
            self.__currentState = self.STATE_NORMAL_OPERATION
            logPrint('Setting current state to: STATE_NORMAL_OPERATION.')
            # Set the operating mode.
            self.__currentMode = self.MODE_WAYPOINT_SEARCH
            logPrint('Setting current mode to: MODE_WAYPOINT_SEARCH.')
            # Set the run mode of the spacial controller
            self.setRunModeSpacial(self.SPACIAL_MODE_EXT_CONTROL)
        return
    
    
    def __normalOperation(self):
        """As we would expect, we'll spend most of our time here. 
        Control actuators, etc..
        """
        # TODO: lots more control flow, everything is driven by spacial updates
        self.__computeSpacialPosition()
        self.__computeLocationDistance()
        self.__computeTurnDirection()
        self.__motorSpeed = self.SPEED_MAX
        #print 'Turn angle:', self.__turnAngle, ', Turn direction:', self.__turnDirection 
        turn = self.__turnAngle * self.TURN_RATIO
        # If we are within a threshold distance from an intermediate location
        # then increment to the next location.
        goalLocation = self.__locations[self.__locationIndex]
        locType = goalLocation[1][0]
        
        if locType == self.LOCATION_TYPE_WAYPOINT \
            and self.__distanceToLocation <= self.THRESHOLD_LOCATION_DISTANCE: 
            self.__locationIndex += 1
        elif locType == self.LOCATION_TYPE_TARGET \
            and self.__distanceToLocation <= self.THRESHOLD_TARGET_DISTANCE:
            self.__currentMode = self.MODE_TARGET_SEARCH
            self.__searchForTarget()
            self.__motorSpeed = self.SPEED_MED
            if self.__targetVisible:
                self.__currentMode = self.MODE_TARGET_TRACKING
                self.__moveTowardTarget()
                #print 'Turn angle:', self.__turnAngle, ', Turn direction:', self.__turnDirection 
                turn = self.__turnAngle * self.TURN_RATIO_VISION
                if self.__distanceToTarget > 0 and self.__distanceToTarget < 30:
                    #logger.info('Approaching target at slow speed.')
                    self.__motorSpeed = self.SPEED_MIN
                if self.__distanceToTarget > 0 and self.__distanceToTarget < 16.0:
                    logPrint('Contacted target, incrementing location index.')
                    self.__locationIndex += 1
                    # Reset our vector location to the next one in the list. 
                    # We know exactly where we are because we contacted the target.
                    self.__utmVector = ()
                    self.__currentMode = self.MODE_WAYPOINT_SEARCH
        if self.__currentMode == self.MODE_WAYPOINT_SEARCH \
            and self.__checkAutoAvoidRequired(): 
                self.setRunModeSpacial(self.SPACIAL_MODE_AUTO_AVOID)
                self.__autoAvoidTimer.resetTimeout(1.0) # 10 seconds.
        elif self.__autoAvoidTimer.checkTimeout() < 0:
            # Revert to mission planner control after a timeout.
            self.setRunModeSpacial(self.SPACIAL_MODE_EXT_CONTROL)
        
        #print 'Turn angle:', self.__turnAngle, ', Turn direction:', self.__turnDirection 
        if self.__spacialUpdateCounter % self.SPEED_UPDATE_INTERVAL == 0 \
            and self.__currentSpacialMode != self.SPACIAL_MODE_AUTO_AVOID:
            if self.__turnDirection == self.TURN_LEFT:
                self.setMotorSpeedLeft(int(self.__motorSpeed - turn))
                self.setMotorSpeedRight(int(self.__motorSpeed))
            elif self.__turnDirection == self.TURN_RIGHT:
                self.setMotorSpeedLeft(int(self.__motorSpeed))
                self.setMotorSpeedRight(int(self.__motorSpeed - turn))
            else:
                self.setMotorSpeedLeft(int(self.__motorSpeed))
                self.setMotorSpeedRight(int(self.__motorSpeed))
        
        # If we are at the last location, then halt.
        if self.__locationIndex >= len(self.__locations):
            # Halt the robot
            logPrint('Exhausted locations to traverse, stopping.')
            self.setRunModeSpacial(self.SPACIAL_MODE_EXT_CONTROL)
            self.setMotorSpeedLeft(0)
            self.setMotorSpeedRight(0)
            self.__currentMode = self.MODE_HALT
            self.shutdown()
        return  
    
    
    def updateSpacial(self, message):
        """All robot control is driven by spacial update message events.
        """
        self.__spacialUpdateCounter += 1
        self.__messageSpacial = message
        self.__prevEncoderCountsLeft = self.__encoderCountsLeft
        self.__prevEncoderCountsRight = self.__encoderCountsRight
        
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
        elif self.__currentState == self.STATE_NORMAL_OPERATION:
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
        
        if len(self.__headingList) > 10:
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
        """ Update the vision information. Can see target...
        """
        self.__messageVision = message
        self.__targetVisible = message.targetShapeVisible
        self.__targetX = message.targetShapeX
        self.__targetY = message.targetShapeY
        
        return
    
    
    def shutdown(self):
        """ Unload the mission planner,
        """
        self.__currentState = self.STATE_SHUTDOWN
        self.__dataProcessor.shutdown()
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
    
    
    def printMag(self, axis=1):
        if self.__messageInertial is not None:
            if axis == 0:
                print self.__messageInertial.magX
            elif axis == 1:
                print self.__messageInertial.magY
            elif axis == 2:
                print self.__messageInertial.magZ
        return
    
    
    def setMotorSpeedLeft(self, speed):
        """speed: signed integer, -10 to 10 are practical values."""
        self.__dataProcessor.sendMessage('s:l:' + str(speed) + ';')
        return
    
    
    def setMotorSpeedRight(self, speed):
        """speed: signed integer, -10 to 10 are practical values."""
        self.__dataProcessor.sendMessage('s:r:' + str(speed) + ';')
        return
    
    
    def setRunModeSpacial(self, mode):
        """mode: run modes are:
        MODE_AUTO_AVOID = 0
        MODE_EXT_CONTROL = 1
        MODE_REMOTE_CONTROL = 2
        """
        if self.__currentSpacialMode != mode:
            logPrint('Setting spacial controller mode: ' + self.SPACIAL_MODE_NAMES[mode])
            self.__dataProcessor.sendMessage('s:m:' + str(mode) + ';')
            self.__currentSpacialMode = mode
        return
    
    
    def showMap(self):
        """""" 
        try:
            self.__staticMap.getMapImage((self.__lat, self.__lon)
                                         , self.__locationsLatLon
                                         , self.__locationName)
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
        # Sum the next vector from the spacial system.
        if len(self.__utmVector) == 0:
            # Set the initial vector and the left and right motor zeros.
            self.__initialEncoderCountsLeft = self.__encoderCountsLeft
            self.__initialEncoderCountsRight = self.__encoderCountsRight
            # Set the UTM spacial coordinate to the n'th location in the
            # mission file.
            self.__vectorPositionX = self.__locations[self.__locationIndex][0][0]
            self.__vectorPositionY = self.__locations[self.__locationIndex][0][1]
            self.__utmVector = (self.__vectorPositionX, self.__vectorPositionY)
        else:
            # Compute the delta counts and average the two, combine this with 
            # the compass direction to produce a vector.
            
            deltaL = self.__encoderCountsLeft - self.__prevEncoderCountsLeft
            deltaR = self.__encoderCountsRight - self.__prevEncoderCountsRight
            deltaCounts = (deltaL + deltaR) / 2.0
            # If we did not move, then we don't need to update this information.
            if deltaCounts == 0:
                return
            
            # Average the the headings in the heading list. Then clear it.
            # If the heading list is empty, then simply use the previous heading.
            headingSum = 0
            for heading in self.__headingList:
                headingSum += heading
            if len(self.__headingList) > 0:
                averageHeading = headingSum / float(len(self.__headingList))
                # Convert the heading into radians.
                self.__averageHeading = averageHeading
            
            # Convert the delta counts to meters.
            
            # Average the current and previous heading if we only got a single
            # heading value.
            if len(self.__headingList) == 1 and self.__prevAverageHeading != 0:
                averageHeading = (averageHeading + self.__prevAverageHeading) / 2.0
                
            print 'Delta Counts: ', deltaCounts, ', Average Heading', averageHeading
            # Add the X and Y values to the UTM vector values.
            # TODO: convert to meters before assignment.
            self.__vectorPositionX += deltaCounts * math.cos(averageHeading)
            self.__vectorPositionY += deltaCounts * math.sin(averageHeading)
            
            
            # Assign a new updated tuple value to the spacial UTM variable.
            self.__utmVector = (self.__vectorPositionX, self.__vectorPositionY)
            self.__headingList = []
            self.__prevAverageHeading = self.__averageHeading
        return
    
    
    def __computeInertialPosition(self):
        # Currently unused
        return
    
    
    def __computeCompositePosition(self):
        # Combine the positions.
        # For now, simply average the GPS and spacial positions. Weight the
        # GPS position by the number of satelites used (above some minimum)
        # and weight the spacial position by the inverse of the distance
        # travelled (since the accuracy of the vector position goes down over
        # time.
        return
    
    
    def __computeLocationDistance(self):
        # Calculate the distance between where we are and where we want to go.
        # For now this is all based off of GPS.
        if len(self.__utmGps) == 0:
            return
        currX = self.__utmGps[0]
        currY = self.__utmGps[1]
        
        goalX = self.__locations[self.__locationIndex][0][0]
        goalY = self.__locations[self.__locationIndex][0][1]
        
        self.__distanceToLocation = math.sqrt(math.pow(goalX - currX, 2) +
                                              math.pow(goalY - currY, 2))
        
        return self.__distanceToLocation
    
    
    def __computeTurnDirection(self):
        """Given our current location and heading, compute the direction we 
        must turn to go towards the next location.
        
        Once again, this is currently based off of GPS, we will eventually
        base these calculations off of the composite position."""
        if len(self.__utmGps) == 0:
            return
        
        currX1 = self.__utmGps[0]
        currY1 = self.__utmGps[1]
        
        currX = self.__vectorPositionX
        currY = self.__vectorPositionY
        # TODO: average these two positions per some weighting scheme.
        
        goalX = self.__locations[self.__locationIndex][0][0]
        goalY = self.__locations[self.__locationIndex][0][1]
        
        heading = self.__headingDegrees
        angle = math.atan2(goalY - currY, goalX - currX)

        if angle < 0.0:
            angle += 360.0
        # Now figure out which is the least angle between the direction we are 
        # heading and the direction we want to face.
        
        # And determine whether we turn right or left.
        turnAngle = abs(heading - angle)
        if turnAngle > 180.0:
            turnAngle = 360.0 - abs(heading - angle)
        
        self.__turnAngle = turnAngle
        
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
    
    # TODO: define search patterns, 
    
    def __searchForTarget(self):
        # Here we execute target search patterns, but for now, do nothing.
        # The targetVisible flag is already set in the updateVision function.
        return
    
    
    def __moveTowardTarget(self):
        # Here we determine which direction to turn to drive toward the cone.
        # If we are close enough, go really slow, we also decide if have 
        # made contact with the target here, basically, when there is a 
        # minimum distance to the target.
        distanceToTarget = self.__messageSpacial.sonarFront
        # basically, we center the target in the image.
        imageMiddleX = 640 / 2.0
        if self.__targetX < imageMiddleX:
            self.__turnDirection = self.TURN_LEFT
        elif self.__targetX > imageMiddleX:
            self.__turnDirection = self.TURN_RIGHT
        else:
            self.__turnDirection = self.TURN_STRAIGHT
            
        pixelError = (1.0 / distanceToTarget) * 100.0
        self.__turnAngle = abs(self.__targetX - imageMiddleX)
        if self.__turnAngle < pixelError: # TODO, scale this by the distance
            # when we are farther away the error will need to be less. But 
            # when we are close, the target occupies more of the FOV. 
            self.__distanceToTarget = distanceToTarget
        else:
            # Target is not being ranged. Set to -1 invalid.
            self.__distanceToTarget = -1
        return
    
    
    def showDebug(self):
        foo = Ui_MainWindow()
        
    
    def printDebug(self):
        #print '\x1b[2J\x1b[H' # This clears the console apparently
        print 'Inertial System Data:'
        print '*********************'
        print 'Magnetometer:'
        print '-------------'
        print 'Heading:', self.__messageInertial.heading
        print 'Accelerometer:'
        print '--------------'
        print 'Pitch:  ', self.__messageInertial.accPitch
        print 'Roll:   ', self.__messageInertial.accRoll
        print 'Speed X:', self.__messageInertial.accSpeedX # First Integral
        print 'Speed Y:', self.__messageInertial.accSpeedY
        print 'Speed Z:', self.__messageInertial.accSpeedZ
        print 'Dist X: ', self.__messageInertial.accDistX # Second Integral
        print 'Dist Y: ', self.__messageInertial.accDistY
        print 'Dist Z: ', self.__messageInertial.accDistZ
        print 'Rate Gyroscope:'
        print '---------------'
        print 'Gyro X: ', self.__messageInertial.gyroDegreesX
        print 'Gyro Y: ', self.__messageInertial.gyroDegreesY
        print 'Gyro Z: ', self.__messageInertial.gyroDegreesZ
        print 'Inertial System Status:'
        print '-----------------------'
        print 'In Motion:      ', self.__messageInertial.inMotion
        print 'Accel Gyro Cal: ', self.__messageInertial.accelGyroCal
        print 'Compass Cal:    ', self.__messageInertial.compassCal
        
        print 'Spacial System Data:'
        print '********************'
        print 'Sonar Distances (cm):'
        print '---------------------'
        print 'Sonar Left:  ', self.__messageSpacial.sonarLeft
        print 'Sonar Front: ', self.__messageSpacial.sonarFront
        print 'Sonar Right: ', self.__messageSpacial.sonarRight
        print 'Sonar Back:  ', self.__messageSpacial.sonarBack
        print 'Encoder Counts:'
        print '---------------'
        print 'Encoder Counts Left: ', self.__messageSpacial.encoderCountsLeft
        print 'Encoder Counts Right:', self.__messageSpacial.encoderCountsRight
        print 'Motor Status:'
        print '-------------'
        print 'Control Variable Left: ', self.__messageSpacial.motorControlVariableLeft
        print 'Set Point Left:        ', self.__messageSpacial.motorSetPointLeft
        print 'Process Variable Left  ', self.__messageSpacial.motorProcessVariableLeft
        print 'Control Variable Right: ', self.__messageSpacial.motorControlVariableRight
        print 'Set Point Right:        ', self.__messageSpacial.motorSetPointRight
        print 'Process Variable Right: ', self.__messageSpacial.motorProcessVariableRight
        print 'Motors Run Mode:     ', self.SPACIAL_MODE_NAMES[self.__messageSpacial.motorsRunMode]
        print 'Motors Enabled:      ', self.__messageSpacial.motorsEnabled
        
        print 'Vector Position Data:'
        print '*********************'
        print 'Vector Position X: ', self.__vectorPositionX
        print 'Vector Position Y: ', self.__vectorPositionY
        
        print 'Camera Tracking Data:'
        print '*********************'
        print 'Target Visible: ', self.__targetVisible
        print 'Target X: ', self.__targetX
        print 'Target Y: ', self.__targetY
        
        print 'GPS Position Data:'
        print '******************'
        print 'Latitude:  ', self.__lat
        print 'Longitude: ', self.__lon
        print 'UTM:  ', repr(self.__utmGps)
        
        print 'Composite Values:'
        print '******************'
        print 'Turn Direction: ', self.__turnDirection
        print 'Turn Magnitude: ', self.__turnAngle
        print 'Speed:  ', self.__motorSpeed
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

def showMap():
    mp.showMap()
    #t = Timer(1.0, show)
    #t.start() 
    return
    
def stat():
    # TODO: print the status of the system.
    pass
    return

def pic():
    # Take a picture and display it.
    #dp.sendMessage('c:p:' + int(show) + ';')
    mp.saveImages()
    return

def magY():
    # Take a picture and display it.
    #dp.sendMessage('c:p:' + int(show) + ';')
    mp.printMag(1)
    return

def left(speed=0):
    # set the left motor speed.
    mp.setMotorSpeedLeft(speed)
    return

def right(speed=0):
    # set the speed of the right motor.
    mp.setMotorSpeedRight(speed)
    return
    
MODE_AUTO_AVOID = 0
MODE_EXT_CONTROL = 1
MODE_REMOTE_CONTROL = 2

def mode(mode):
    # set the speed of the right motor.
    mp.setRunModeSpacial(mode)
    return

def speed(speed):
    left(speed)
    right(speed)
    return

def turn(speed, direction):
    speedLeft = speed + direction
    speedRight = speed - direction
    left(speedLeft)
    right(speedRight)
    return

def magX():
    mp
    return

def showCamera():
    mp.showCamera()
    return

def showDebug():
    mp.showDebug()
    return

def printDebug(period=0):
    while True:
        mp.printDebug()
        if period == 0: break
        time.sleep(period)
    return


    