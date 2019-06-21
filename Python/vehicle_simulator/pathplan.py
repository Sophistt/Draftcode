import math
from copy import deepcopy


class Point:
    def __init__(self, p):
        self.x = p[0]
        self.y = p[1]

    def __repr__(self):
        return '({}, {})'.format(self.x, self.y)


class PathPlan:
    def __init__(self,
                 direction=0):
        self.goalDirection = direction

    @staticmethod
    def hermite(currentPoint, goalPoint, goalDir):
        curPoint = Point(currentPoint)
        goalPoint = Point(goalPoint)
        direction = goalDir

        # Compute control points
        curPoint_1_x = curPoint.x + 6
        curPoint_1_y = curPoint.y
        curPoint_1 = Point((curPoint_1_x, curPoint_1_y))

        goalPoint_1_x = goalPoint.x - 7 * math.cos(direction)
        goalPoint_1_y = goalPoint.y
        goalPoint_1 = Point((goalPoint_1_x, goalPoint_1_y))

        cp = [curPoint, curPoint_1, goalPoint_1, goalPoint]

        # Compute bezier curve according to 4 control points and rank
        def bezier(control_points, rank):
            temp_points = []
            planpath = []

            for i in range(200):
                u = i / 199.0
                for index in range(len(control_points)):
                    temp_points.append(deepcopy(control_points[index]))

                j = 1
                while j < rank:
                    k = 0
                    while k <= (rank - j):
                        temp_points[k].x = (1 - u) * temp_points[k].x + u * temp_points[k + 1].x
                        temp_points[k].y = (1 - u) * temp_points[k].y + u * temp_points[k + 1].y
                        k += 1
                    j += 1
                planpath.append(deepcopy(temp_points[0]))

            return planpath

        return bezier(cp, 3)


if __name__ == "__main__":
    PathPlan.test((5, 0))
    PathPlan.testPrint()
