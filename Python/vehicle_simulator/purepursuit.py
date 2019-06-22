import sys
import math

from copy import deepcopy

from pathplan import Point


# PurePursuit class calculates the reasonable tire angle to control the vehicle
class PurePursuit(object):
    def __init__(self):
        self.lookaheadDistance = 11
        self.wheelBase = 4

    
    # Essential method
    # Function: choose a suitable point according to look ahead distance in track points list
    # and use ackermann geometry to calculate the reasonable tire angle.
    def trajectoryTracking(self, pathPoints, currentPoint, direction):
        tmpDistance = 50
        trackPoint = {}

        for i in range(200):
           calDistance = abs(pathPoints[i].x - currentPoint.x - self.lookaheadDistance)
           if calDistance < tmpDistance:
               tmpDistance = calDistance
               trackPoint = deepcopy(pathPoints[i])
        
        goalPoint = Point(self.pixelCoordsToVehicleCoords(currentPoint, trackPoint, direction))
        currentPoint = Point((0, 0))

        kappa = self.computeCurvature(currentPoint, goalPoint)
        return self.convertCurvatureToTireAngle(kappa) * 180 / math.pi
    
    
    # Compute curvature of the curve connecting two points
    def computeCurvature(self, currentPoint, trackPoint):
        deltaX = (trackPoint.x - currentPoint.x) * 1.0
        deltaY = (trackPoint.y - currentPoint.y) * 1.0
        chordLength = deltaX * deltaX + deltaY * deltaY
        
        return 2.0 * deltaX / chordLength 
   
    
    # Compute tire angle according to curvature and the wheelbase of vehicle
    def convertCurvatureToTireAngle(self, kappa):
        return math.atan(self.wheelBase * kappa)

    
    # Get Angle between a line and x axis in pixel coordiante
    # The line connects currentPoint and trackPoint
    def getAngle(self, currentPoint, trackPoint):
        if trackPoint.x == currentPoint.x:
            if trackPoint.y < currentPoint.y:
                return math.pi / 2
            elif trackPoint.y == currentPoint.y:
                return 0
            else:
                return math.pi * 3 / 2

        angle = math.atan((currentPoint.y - trackPoint.y) / (trackPoint.x - currentPoint.x))
        if trackPoint.x - currentPoint.x < 0:
            angle = angle + math.pi

        return angle 

    # Get distance between two Points
    def getDistance(self, currentPoint, trackPoint):
        deltaX = (trackPoint.x - currentPoint.x) * 1.0
        deltaY = (trackPoint.y - currentPoint.y) * 1.0

        return math.sqrt(deltaX * deltaX + deltaY * deltaY)


    # Convert points in pixel coordinate to vehicle coordinate
    def pixelCoordsToVehicleCoords(self, currentPoint, trackPoint, direction):
        dis = self.getDistance(currentPoint, trackPoint)
        angle = self.getAngle(currentPoint, trackPoint)
        radAngle = math.pi / 2 - math.radians(direction) + angle

        vehicleX = dis * math.cos(radAngle)
        vehicleY = dis * math.sin(radAngle)

        return (vehicleX, vehicleY)
        
        



def main():
    purepursuit = PurePursuit()
    curPoint = Point((0, 0))
    aPoint = Point((-1, 1))
    
    print(purepursuit.pixelCoordsToVehicleCoords(curPoint, aPoint, 0))



if __name__ == "__main__":
    main()


