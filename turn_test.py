
def turn1(heading, angle):
    headingMinusAngle = heading - angle
    if headingMinusAngle < 0:
        if abs(headingMinusAngle) < 180:
            print 'TURN_LEFT'
        else:
            print 'TURN_RIGHT'
    elif headingMinusAngle > 0:
        if abs(headingMinusAngle) < 180:
            print 'TURN_RIGHT'
        else:
            print 'TURN_LEFT'
    else:
        print 'TURN_STRAIGHT'


def turn2(heading, angle):
    if angle - heading < 0 and abs(angle - heading) >= 180:
        print 'TURN_LEFT'
    elif angle - heading > 0 and abs(angle - heading) >= 180:
        print 'TURN_RIGHT'
    elif angle - heading < 0 and abs(angle - heading) < 180:
        print 'TURN_RIGHT'
    elif angle - heading > 0 and abs(angle - heading) < 180:
        print 'TURN_LEFT'
    else:
        print 'TURN_STRAIGHT'


for i in range(0, 360):
    for j in range(0, 360):
        turn2(i, j)
