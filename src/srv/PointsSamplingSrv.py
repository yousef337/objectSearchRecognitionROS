#!/usr/bin/env python3
import rospy
import numpy as np
from math import floor
from object_search.srv import (
    LocationSampling,
    LocationSamplingRequest,
    LocationSamplingResponse,
)


def findNearestPoint(point, cordsLst):
    maxProjScope = max(point[0], point[1])

    for i in range(maxProjScope):
        # check left stride
        for nextY in range(min(i + 1, point[1])):
            if point[0] - i < len(cordsLst) and point[1] - nextY < len(
                cordsLst[0]
            ):
                if cordsLst[point[0] - i][point[1] - nextY] == 1:
                    return [point[0] - i, point[1] - nextY]

        # check upper stride
        for nextX in range(min(i, point[0])):
            if point[0] - nextX < len(cordsLst) and point[1] - i < len(
                cordsLst[0]
            ):
                if cordsLst[point[0] - nextX][point[1] - i] == 1:
                    return [point[0] - nextX, point[1] - i]

    return [None, None]


def isRightPossible(x, y, sampledRoom):
    for i in range(x + 1, len(sampledRoom)):
        if sampledRoom[i][y] == 1:
            return True

    return False


def sampleRoomCB(req: LocationSamplingRequest) -> LocationSamplingResponse:
    x = 0
    y = 0
    res = LocationSamplingResponse()
    sampledRoom = np.array(req.rasterizedRoom).reshape(
        req.deltaBlock[0], req.deltaBlock[1]
    )

    yPadCounter = 0

    while x < req.deltaBlock[0] and y < req.deltaBlock[1]:
        # go to the next first in-room cords
        cord = [None, None]
        yPad = floor(5 * req.visionScope * yPadCounter)
        for i in range(x, req.deltaBlock[0]):
            for j in range(yPad, req.deltaBlock[1]):
                if sampledRoom[i][j] == 1 and cord == [None, None]:
                    cord = [i, j]

        if cord == [None, None]:
            break

        # get next cord
        nextX = floor(cord[0] + req.visionScope)
        nextY = floor(cord[1] + req.visionScope)

        nearestCurrent = findNearestPoint([nextX, nextY], sampledRoom)
        if nearestCurrent != [None, None]:
            sampledRoom[nearestCurrent[0]][nearestCurrent[1]] = 2

        # move current cord to the next one
        x = floor(nearestCurrent[0] + 2 * req.visionScope)
        y = nearestCurrent[1]

        if not isRightPossible(x, y, sampledRoom):
            x = 0
            y += floor(5 * req.visionScope)
            yPadCounter += 1

        if y > len(sampledRoom[1]):
            break

    res.sampledRoom = list(sampledRoom.flatten())
    return res
