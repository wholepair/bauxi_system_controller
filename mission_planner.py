"""
The mission planner interprets mission directives and state information
in order to execute its mission.

Directives include: way-points (mandatory and optional)

Way-point Descriptors: navigation-point, target-point, finish-point, etc..

Sequencing: order to achieve way-points

Behaviors: what to do at target way-point, contact, dwell, repetition:
           to achieve way-points a single time only or in a loop.


TODO: Implement calibration state machine and process.
"""

import sys, math
from lxml import etree
import utm
import data_processor
from data_processor import DataProcessor
from static_maps import StaticMap

logger = data_processor.logger

class MissionLocations(object):
    """ Things to remember:
    Remove the xmlns attribute, it breaks tag string compare operations
    - Add type and handle data to the rtept point list.
    - The 'type' attribute is new and different than Sputnik's 'note' attribute.
      (but it is uesd for more or less the same purpose).
    - 
    """
    
    def __init__(self, fileName):
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
                        logger.info('Lat/Lon Location: ' + repr(latLonLocation))
                        self.__locationsLatLon.append(latLonLocation)
                        utmLocation = utm.from_latlon(lat, lon)
                        self.__locationsUtm.append((utmLocation, locType, handle))
                        logger.info('UTM Location: ' + repr(utmLocation))
        return
    
    
    @property
    def locationsUtm(self):
        return self.__locationsUtm
    
    @property
    def locationsLatLon(self):
        return self.__locationsLatLon
    
    @property
    def rteName(self):
        return self.__rteName
    

class MissionPlanner(object):
    
    # Mission planner state list:
    STATE_UNKNOWN = -1
    STATE_STARTUP = 0
    STATE_SELF_TEST = 1
    STATE_NORMAL_OPERATION = 2
    STATE_FAILED = 3
    STATE_DEVELOPMENT = 4
    STATE_CALIBRATION = 5
    STATE_SHUTDOWN = 6
    
    STATE_NAMES = { STATE_UNKNOWN     : 'STATE_UNKNOWN'
                  , STATE_STARTUP     : 'STATE_STARTUP'
                  , STATE_SELF_TEST   : 'STATE_SELF_TEST'
                  , STATE_NORMAL_OPERATION : 'STATE_NORMAL_OPERATION'
                  , STATE_FAILED      : 'STATE_FAILED'
                  , STATE_DEVELOPMENT : 'STATE_DEVELOPMENT'
                  , STATE_CALIBRATION : 'STATE_CALIBRATION'
                  , STATE_SHUTDOWN    : 'STATE_SHUTDOWN' }
    
    __currentState = STATE_UNKNOWN
    logger.info('Setting current state to: STATE_UNKNOWN.')
    
    # Normal operating modes, may also be used in the development state:
    MODE_UNKNOWN = -1
    MODE_WAYPOINT_SEARCH = 0
    MODE_TARGET_SEARCH = 1
    MODE_TARGET_TRACKING = 2
    MODE_HALT = 3
    
    MODE_NAMES = { MODE_UNKNOWN         : 'MODE_UNKNOWN'
                 , MODE_WAYPOINT_SEARCH : 'MODE_WAYPOINT_SEARCH'
                 , MODE_TARGET_SEARCH   : 'MODE_TARGET_SEARCH'
                 , MODE_TARGET_TRACKING : 'MODE_TARGET_TRACKING'
                 , MODE_HALT            : 'MODE_HALT' }
    
    __currentMode = MODE_UNKNOWN
    logger.info('Setting current mode to: MODE_UNKNOWN.')
    
    SPACIAL_MODE_AUTO_AVOID = 0
    SPACIAL_MODE_EXT_CONTROL = 1
    SPACIAL_MODE_REMOTE_CONTROL = 2
    
    SPACIAL_MODE_NAMES = { SPACIAL_MODE_AUTO_AVOID     : 'SPACIAL_MODE_AUTO_AVOID'
                         , SPACIAL_MODE_EXT_CONTROL    : 'SPACIAL_MODE_EXT_CONTROL'
                         , SPACIAL_MODE_REMOTE_CONTROL : 'SPACIAL_MODE_REMOTE_CONTROL' }
    
    __currentSpacialMode = SPACIAL_MODE_AUTO_AVOID
    
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
    
    LOCATION_TYPE_START = 'start'
    LOCATION_TYPE_WAYPOINT = 'waypoint'
    LOCATION_TYPE_TARGET = 'target'
    LOCATION_TYPE_FINISH = 'finish' # Don't use this one.
    
    # We always start at the start location :).
    __curentLocationType = LOCATION_TYPE_START
    
    FAILED_STATE_COUNT_LIMIT = 100
    
    __staticMap = StaticMap()
    
    def __init__(self):
        """ Constructor.
        """
        # The target location list is a list of locations the robot must go.
        # Start, location 1, 2, 3, etc..
        # Format: name: (lat, lon), type
        # The location type specifies what we must do once we get to the location.
        # The location types are: start, waypoint, target, and finish.
        # The start location type is the starting location.
        # The waypoint location type is an intermediate location, used to 
        # direct the robot away from hazards, etc..
        # 
        # Get the mission file and parse it.
        loc = MissionLocations('mission.gpx')
        self.__locations = loc.locationsUtm
        self.__locationsLatLon = loc.locationsLatLon
        self.__locationName = loc.rteName
        
        
        # The distance to the next/current location in the location list.
        self.__distanceToLocation = sys.maxint
        # A boolean flag, True when the vision system has spotted the target.
        self.__targetVisible = False
        # The distance to the target, measured by the range sensor
        self.__distanceToTarget = sys.maxint
        self.__failedStateCounter = 0
        
        self.__utmGps = ()
        self.__utmSpacial = ()
        
        self.__initialEncoderCountsLeft = 0
        self.__initialEncoderCountsRight = 0
        
        self.__encoderCountsLeft = 0
        self.__encoderCountsRight = 0
        
        self.__prevEncoderCountsLeft = 0
        self.__prevEncoderCountsRight = 0
        
        self.__averageHeading = 0.0
        self.__prevAverageHeading = 0.0
        self.__headingList = []
        
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
        
        # The first location should be 0, but we don't have the spacial system 
        # up and running so start by going to the next point.
        self.__locationIndex = 1
        logger.info('Current Goal Location: ' + repr(self.__locations[0]))
        
        self.__currentState = self.STATE_STARTUP
        logger.info('Setting current state to: STATE_STARTUP.')
        # We remain in the startup state until we get our first message. 
        # Then we transition to the STATE_SELF_TEST state.
        self.__dataProcessor = DataProcessor(parent=True, missionPlanner=self)

        return
    
    
    def updateSpacial(self, message):
        """ 
        """
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
            logger.info('Setting current state to: STATE_SELF_TEST.')
        elif self.__currentState == self.STATE_SELF_TEST:
            # Start a counter and if a limit is exceeded, then go into
            # the failed state.
            self.__failedStateCounter += 1
            if self.__failedStateCounter > self.FAILED_STATE_COUNT_LIMIT:
                self.__currentState = self.STATE_FAILED
                logger.warning('Failed state counter exceeded failed state count limit')
                logger.info('Setting current state to: STATE_FAILED.')
                print 'Failed state counter exceeded failed state count limit'
                print 'Setting current state to: STATE_FAILED.'
                logger.info('Inertial System: ' + repr(self.__messageInertial))
                logger.info('GPS System: ' + repr(self.__messageGps))
                logger.info('Vision System: ' + repr(self.__messageVision))
                print 'Inertial System:', self.__messageInertial
                print 'GPS System:', self.__messageGps
                print 'Vision System:', self.__messageVision
            if self.__messageInertial is not None \
                and self.__messageGps is not None \
                and self.__messageVision is not None:
                self.__currentState = self.STATE_NORMAL_OPERATION
                logger.info('Setting current state to: STATE_NORMAL_OPERATION.')
                # Set the operating mode.
                self.__currentMode = self.MODE_WAYPOINT_SEARCH
                logger.info('Setting current mode to: MODE_WAYPOINT_SEARCH.')
                # Set the run mode of the spacial controller
                logger.info('Setting spacial controller mode: SPACIAL_MODE_EXT_CONTROL')
                self.setRunModeSpacial(self.SPACIAL_MODE_EXT_CONTROL)
        
        # TODO: lots more control flow, everything is driven by spacial updates
        #self.__computeSpacialPosition()
        
        self.__computeLocationDistance()
        
        self.__computeTurnDirection()
        
        speed = self.SPEED_MAX
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
            
            if self.__currentMode != self.MODE_TARGET_SEARCH:
                self.__currentMode = self.MODE_TARGET_SEARCH
            
            self.__searchForTarget()
            speed = self.SPEED_MED
            if self.__targetVisible:
                
                if self.__currentMode != self.MODE_TARGET_TRACKING:
                    self.__currentMode = self.MODE_TARGET_TRACKING
                
                self.__moveTowardTarget()
                #print 'Turn angle:', self.__turnAngle, ', Turn direction:', self.__turnDirection 
                turn = self.__turnAngle * self.TURN_RATIO_VISION
                if self.__distanceToTarget > 0 and self.__distanceToTarget < 30:
                    #logger.info('Approaching target at slow speed.')
                    speed = self.SPEED_MIN
                if self.__distanceToTarget > 0 and self.__distanceToTarget < 16.0:
                    logger.info('Contacted target, incrementing location index.')
                    self.__locationIndex += 1
                    self.__currentMode = self.MODE_WAYPOINT_SEARCH
                    
                
        if self.__currentMode == self.MODE_WAYPOINT_SEARCH:
            if self.__messageSpacial.sonarFront < 50 \
                or self.__messageSpacial.sonarLeft < 30 \
                or self.__messageSpacial.sonarRight < 30:
                self.setRunModeSpacial(self.SPACIAL_MODE_AUTO_AVOID)
        else:
            self.setRunModeSpacial(self.SPACIAL_MODE_EXT_CONTROL)
        # If we are within a threshold distance to a target location, then
        # begin looking for it/approaching it. 
        
        # If we contacted the target, then increment to the next location
        
        #print 'Turn angle:', self.__turnAngle, ', Turn direction:', self.__turnDirection 
        
        if self.__turnDirection == self.TURN_LEFT:
            self.setMotorSpeedLeft(int(speed - turn))
            self.setMotorSpeedRight(int(speed))
        elif self.__turnDirection == self.TURN_RIGHT:
            self.setMotorSpeedLeft(int(speed))
            self.setMotorSpeedRight(int(speed - turn))
        else:
            self.setMotorSpeedLeft(int(speed))
            self.setMotorSpeedRight(int(speed))
        
        # If we are at the last location, then halt.
        if self.__locationIndex >= len(self.__locations):
            # Halt the robot
            self.setMotorSpeedLeft(0)
            self.setMotorSpeedRight(0)
            self.__currentMode = self.MODE_HALT
            self.shutdown()
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
        self.CURRENT_STATE = self.STATE_SHUTDOWN
        self.__dataProcessor.shutdown()
        return
    
    
    def takePicture(self):
        print 'Mission Planner: send picture taking message.'
        self.__dataProcessor.sendMessage('c:pic')
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
        """ speed: signed integer, -10 to 10 are practical values.
        """
        self.__dataProcessor.sendMessage('s:l:' + str(speed) + ';')
        return
    
    
    def setMotorSpeedRight(self, speed):
        """ speed: signed integer, -10 to 10 are practical values.
        """
        self.__dataProcessor.sendMessage('s:r:' + str(speed) + ';')
        return
    
    
    def setRunModeSpacial(self, mode):
        """ mode: run modes are:
        MODE_AUTO_AVOID = 0
        MODE_EXT_CONTROL = 1
        MODE_REMOTE_CONTROL = 2
        """
        self.__dataProcessor.sendMessage('s:m:' + str(mode) + ';')
        return
    
    
    def showMap(self):
        """
        """ 
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
        if len(self.__utmSpacial) == 0:
            # Set the initial vector and the left and right motor zeros.
            self.__initialEncoderCountsLeft = self.__encoderCountsLeft
            self.__initialEncoderCountsRight = self.__encoderCountsRight
            # Set the UTM spacial coordinate to the first location in the
            # mission file.
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
            x = deltaCounts * math.cos(averageHeading)
            y = deltaCounts * math.sin(averageHeading)
            
            # Add the X and Y values to the UTM Spacial tuple values and assign
            # a new updated tuple value to the spacial UTM variable.
            
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
        # Given our current location and heading, compute the direction we 
        # must turn to go towards the next location.
        # Once again, this is currently based off of GPS, we will eventually
        # base these calculations off of the composite position.
        if len(self.__utmGps) == 0:
            return
        currX = self.__utmGps[0]
        currY = self.__utmGps[1]
        
        goalX = self.__locations[self.__locationIndex][0][0]
        goalY = self.__locations[self.__locationIndex][0][1]
        
        heading = self.__headingDegrees
        angle = math.atan2(goalY - currY, goalX - currX)

        if angle < 0.0:
            angle += 360.0
        # Now figure out which is the least angle between the direction we are 
        # heading and the direction we want to face.
        
        # And figure out whether we turn right or left.
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
    
    
    # Properties: 
    @property
    def lat(self):
        return self.__lat
    
    @property
    def lon(self):
        return self.__lon
    
    
    @property
    def currentState(self):
        return self.CURRENT_STATE
    
    @property
    def currentStateName(self):
        return self.STATE_NAMES[self.CURRENT_STATE]
    
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
    mp.takePicture()
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


    