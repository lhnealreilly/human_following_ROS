import numpy as np
import math


#Find the closes point on a line segment to a given point
def closestPointOnLine(lineStart, lineEnd, point):
    #Calculate the length of the line
    lineLength = math.sqrt((lineEnd[0] - lineStart[0]) ** 2 + (lineEnd[1] - lineStart[1]) ** 2)

    # Calculate the area of the triangle formed by the line and the point
    area = abs((point[0] - lineStart[0]) * (lineEnd[1] - lineStart[1]) -(point[1] - lineStart[1]) * (lineEnd[0] - lineStart[0])) / 2

    # Calculate the distance between the point and the line
    distance = (2 * area) / lineLength

    # Calculate the parameter t
    t =((point[0] - lineStart[0]) * (lineEnd[0] - lineStart[0]) +(point[1] - lineStart[1]) * (lineEnd[1] - lineStart[1])) /lineLength ** 2

    # Find the closest point on the line
    closestPoint = [lineStart[0] + t * (lineEnd[0] - lineStart[0]),lineStart[1] + t * (lineEnd[1] - lineStart[1])]

    # If t is less than 0, the closest point is lineStart
    if (t < 0):
        closestPoint = lineStart

    # If t is greater than 1, the closest point is lineEnd
    if (t > 1):
        closestPoint = lineEnd

    return closestPoint
