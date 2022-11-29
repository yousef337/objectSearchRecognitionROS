#!/usr/bin/env python3
import rospy
from object_search.srv import LocationSampling, LocationSamplingRequest, LocationSamplingResponse

def findNearestPoint(point, cordsLst):
    maxProjScope = max(point[0], point[1])

    for i in range(maxProjScope):
        # check left stride
        for nextY in range(min(i + 2, point[1])):
            if cordsLst[point[0] - i][point[1] + nextY] == 1:
                return [point[0] - i, point[1] + nextY]
        
        # check upper stride
        for nextX in range(min(i + 1, point[0])):
            if cordsLst[point[0] - i + nextX][point[1] + i] == 1:
                return [point[0] - i + nextX, point[1] + i]

    return [None, None]


def sampleRoomCB(req: LocationSamplingRequest) -> LocationSamplingResponse:
    x, y = 0
    res = LocationSamplingResponse()
    res.sampledRoom = req.rasterizedRoom

    while x < req.deltaBlock[0] and y < req.deltaBlock[1]:
        # go to the next first in-room cords
        cord = [None, None, None]
        for i in range(x, req.deltaBlock[0]):
            for j in range(y, req.deltaBlock[1]):
                if (req.rasterizedRoom[i][j] == 1):
                    cord = req.rasterizedRoom[i][j]

        if cord == [None, None, None]:
            return res
        
        # get next cord
        nextX = cord[0] + 0.5 * req.visionScope
        nextY = cord[1] + 0.5 * req.visionScope

        nearestCurrent = findNearestPoint([nextX, nextY], req.rasterizedRoom)
        if nearestCurrent == [None, None]:
            return res
        else:
            res.sampledRoom[nearestCurrent[0]][nearestCurrent[1]] = 2
        
        # move current cord to the next one
        x *= 4 * req.visionScope
        y *= 4 * req.visionScope
        nextCord = findNearestPoint([x, y], req.rasterizedRoom)
        if nextCord != [None, None]:
            x = nextCord[0]
            y = nextCord[1]
            
