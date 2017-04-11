#!/usr/bin/python

"""
Derived from Paul Bourke's Equation of a Circle from 3 Points (2 dimensions)
http://paulbourke.net/geometry/circlesphere/
"""
import math

class Point(object):
    
    def __init__(self, coor1=None, coor2=None, coor3=None):
        """
        """
        if isinstance(coor1, (int, long, float, complex)):
            self.__x = coor1
            self.__y = coor2
            self.__z = coor3
        elif coor1 is not None:
            self.__x = coor1.x
            self.__y = coor1.y
            self.__z = coor1.z
        else:
            self.__x = 0.0
            self.__y = 0.0
            self.__z = 0.0
        return
    
    
    def setX(self, x):
        """
        """
        self.__x = x
        return
    
        
    def setY(self, y):
        """
        """
        self.__y = y
        return
    
        
    def setZ(self, z):
        """
        """
        self.__z = z
        return
    
    
    @property
    def x(self):
        return self.__x
    
    
    @property
    def y(self):
        return self.__y
    
    
    @property
    def z(self):
        return self.__z
    
    

class Circle(object):
    """
    """

    # public:

    # double GetRadius();
    def getRadius(self):
        """
        """
        pass
    
    # Point* GetCenter();
    def getCenter(self):
        """
        """
        pass
    
    # // p1, p2, p3 are co-planar
    # Circle(Point *p1, Point *p2, Point *p3);    
    def __init__(self, V1=None, V2=None, V3=None):
        """
        """
        self.__radius = -1
        self.__center = Point()
        pt1 = Point(V1)
        pt2 = Point(V2)
        pt3 = Point(V3)
        
        if not self.__isPerpendicular(pt1, pt2, pt3):
            self.__calcCircle(pt1, pt2, pt3)
        elif not self.__isPerpendicular(pt1, pt3, pt2):
            self.__calcCircle(pt1, pt3, pt2)
        elif not self.__isPerpendicular(pt2, pt1, pt3):
            self.__calcCircle(pt2, pt1, pt3)
        elif not self.__isPerpendicular(pt2, pt3, pt1):
            self.__calcCircle(pt2, pt3, pt1)
        elif not self.__isPerpendicular(pt3, pt2, pt1):
            self.__calcCircle(pt3, pt2, pt1)
        elif not self.__isPerpendicular(pt3, pt1, pt2):
            self.__calcCircle(pt3, pt1, pt2)
        else: 
            print "\nThe three points are perpendicular to axis\n"
            # pt1->trace(); pt2->trace(); pt3->trace();
            self.__radius = -1
        del pt1
        del pt2
        del pt3
        return 
    
    
    # Circle();
    
    # virtual ~Circle();

    # private:

    # double CalcCircle(Point *pt1, Point *pt2, Point *pt3);
    def __calcCircle(self, pt1, pt2, pt3):
        """
        """
        
        yDelta_a = pt2.y - pt1.y
        xDelta_a = pt2.x - pt1.x
        yDelta_b = pt3.y - pt2.y
        xDelta_b = pt3.x - pt2.x
    
        if abs(xDelta_a) <= 0.000000001 and abs(yDelta_b) <= 0.000000001:
            print "Calculate circle \n"
            self.__center.setX(0.5 * (pt2.x + pt3.x))
            self.__center.setY(0.5 * (pt1.y + pt2.y))
            self.__center.setZ(pt1.z)
            self.__radius = self.__distance(self.__center, pt1) # calc. radius
            # TRACE(" Center: %f %f %f\n", m_Center.x(), m_Center.y(), m_Center.z());
            # TRACE(" radius: %f %f %f\n", length(&m_Center,pt1), length(&m_Center,pt2),length(&m_Center,pt3));
            return self.__radius
    
        # IsPerpendicular() assure that xDelta(s) are not zero
        aSlope = yDelta_a / xDelta_a # 
        bSlope = yDelta_b / xDelta_b
        if abs(aSlope - bSlope) <= 0.000000001:    # checking whether the given points are colinear.     
            print "The three points are colinear\n"
            return -1

        # Calculate circle center:
        self.__center.setX((aSlope * bSlope * (pt1.y - pt3.y) + bSlope \
                           * (pt1.x + pt2.x) - aSlope * (pt2.x + pt3.x)) \
                           / (2 * (bSlope - aSlope)))
        
        self.__center.setY(-1 * (self.__center.x - (pt1.x + pt2.x) / 2) \
                           / aSlope + (pt1.y + pt2.y / 2))
        
        self.__center.setZ(pt1.z)

        self.__radius = self.__distance(self.__center, pt1) # calc. radius
        
        # TRACE(" Center: %f %f %f\n", m_Center.x(), m_Center.y(), m_Center.z());
        # TRACE(" radius: %f %f %f\n", length(&m_Center,pt1), length(&m_Center,pt2),length(&m_Center,pt3));
        return self.__radius
    
    
    # Boolean IsPerpendicular(Point *pt1, Point *pt2, Point *pt3);
    def __isPerpendicular(self, pt1, pt2, pt3):
        """ Check the given point are perpendicular to x or y axis
        """
        
        yDelta_a = pt2.y - pt1.y
        xDelta_a = pt2.x - pt1.x
        yDelta_b = pt3.y - pt2.y
        xDelta_b = pt3.x - pt2.x
    
        # TRACE(" yDelta_a: %f xDelta_a: %f \n",yDelta_a,xDelta_a);
        # TRACE(" yDelta_b: %f xDelta_b: %f \n",yDelta_b,xDelta_b);

        # checking whether the line of the two points are vertical
        if abs(xDelta_a) <= 0.000000001 and abs(yDelta_b) <= 0.000000001:
            print "The points are pependicular and parallel to x-y axis\n"
            return False

        if abs(yDelta_a) <= 0.0000001:
            # TRACE(" A line of two point are perpendicular to x-axis 1\n");
            return True
        
        elif abs(yDelta_b) <= 0.0000001:
            # TRACE(" A line of two point are perpendicular to x-axis 2\n");
            return True
        elif abs(xDelta_a) <= 0.000000001:
            # TRACE(" A line of two point are perpendicular to y-axis 1\n");
            return True
        elif abs(xDelta_b) <= 0.000000001:
            # TRACE(" A line of two point are perpendicular to y-axis 2\n");
            return True
        else:
            return False
    
    
    def __distance(self, p1, p2):
        """ Compute the distance between the two points
        """
        return math.sqrt((p1.x - p2.x)**2.0 + (p1.y - p2.y)**2.0 + (p1.z - p2.z)**2.0)
    
    # double m_dRadius;
    @property
    def radius(self):
        return self.__radius
    
    
    # Point m_Center;
    @property
    def center(self):
        return self.__center
    
    
# -994.91    -552.66    627.23
# -900.43    -458.9    597.36
# -848.85    -779.58    544.97

p1 = Point(-535.14, -306.46, 0)
p2 = Point(-807.64, -311.12, 0)
p3 = Point(-735.68, -575.96, 0)

foo = Circle(p1, p2, p3)

print 'Center (x, y, z):', foo.center.x, foo.center.y, foo.center.z
print 'Radius:', foo.radius

