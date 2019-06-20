import sys
import math





class PurePursuit(object):
    def __init__(self):
        self.lookaheadDistance = 50
        self.wheelBase = 15


    
    def trajectoryTracking(self, pathPoints, currentPoint, direction):
        tmpDistance = 50
        trackPoint = {}

        for i in range(200):
           calDistance = abs(pathPoints[i]['x'] - currentPoint['x'] - self.lookaheadDistance)
           if calDistance < tmpDistance:
               tmpDistance = calDistance
               trackPoint = dict(pathPoints[i])
        
        goalPoint = self.pixelCoordsToVehicleCoords(trackPoint, currentPoint, direction)
        
        return self.convertCurvatureToTireAngle(kappa) * 180 / math.pi
    
    
    def computeCurvature(self, trackPoint, currentPoint):
        deltaX = (trackPoint['x'] - currentPoint['x']) * 1.0
        deltaY = (trackPoint['y'] - currentPoint['y']) * 1.0
        chordLength = deltaX * deltaX + deltaY * deltaY
        
        return 2.0 * deltaX / chordLength 
    
    def convertCurvatureToTireAngle(self, kappa):
        return math.atan(self.wheelBase * kappa)


    def getAngle(self, trackPoint, currentPoint):
        if trackPoint['x'] == currentPoint['x']:
            if trackPoint['y'] < currentPoint['y']:
                return math.pi / 2
            elif trackPoint['y'] == currentPoint['y']:
                return 0
            else:
                return math.pi * 3 / 2

        angle = math.atan((currentPoint['y'] - trackPoint['y']) / (trackPoint['x'] - currentPoint['x']))
        if trackPoint['x'] - currentPoint['x'] < 0:
            angle = angle + math.pi

        return angle 


    def getDistance(self, trackPoint, currentPoint):
        deltaX = (trackPoint['x'] - currentPoint['x']) * 1.0
        deltaY = (trackPoint['y'] - currentPoint['y']) * 1.0

        return math.sqrt(deltaX * deltaX + deltaY * deltaY)


    def pixelCoordsToVehicleCoords(self, trackPoint, currentPoint, direction):
        dis = self.getDistance(trackPoint, currentPoint)
        angle = self.getAngle(trackPoint, currentPoint)
        radAngle = math.pi / 2 - math.radians(direction) + angle

        vehicleX = dis * math.cos(radAngle)
        vehicleY = dis * math.sin(radAngle)

        return {'x': vehicleX, 'y': vehicleY}
        
        



def main():
    purepursuit = PurePursuit()

    localdict = {'x': 400, 'y': 400}
    ap_dict = {'x': 400, 'y': 300}

    print(purepursuit.PixelCoordsToVehicleCoords(ap_dict, localdict, 45))


if __name__ == "__main__":
    main()


