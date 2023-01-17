#!/usr/bin/env python3
import rospy
import numpy as np
from math import floor
from object_search.srv import (
    LocationSampling,
    LocationSamplingRequest,
    LocationSamplingResponse,
)
from settings import SCOPE_MULTIPLIER


def findNearestPoint(point, cordsLst):
    maxProjScope = int(max(point[0], point[1]))

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


def isCovered(x, y, rasRoom, visionRange):
    for i in range(
        max(0, x - visionRange), min(len(rasRoom), x + visionRange + 1)
    ):
        for j in range(
            max(0, y - visionRange), min(len(rasRoom[0]), y + visionRange + 1)
        ):
            if rasRoom[i][j] == 2:
                return True

    return False


def scoreSampling(rasRoom, visionRange):
    total = 0
    covered = 0

    for x in range(len(rasRoom)):
        for y in range(len(rasRoom[0])):
            if rasRoom[x][y] == 0 or isCovered(x, y, rasRoom, visionRange):
                covered += 1

            total += 1

    return float(covered) / total


def getNextPoint(x, visionScope, yPadCounter, sampledRoom, deltaBlock):
    yPad = int(floor(visionScope * yPadCounter))
    for i in range(x, deltaBlock[0]):
        for j in range(yPad, deltaBlock[1]):
            if sampledRoom[i][j] == 1:
                return [i, j]

    return [None, None]


def sampleRoomCB(req):
    x = 0
    y = 0
    cords = []
    res = LocationSamplingResponse()
    sampledRoom = np.array(req.rasterizedRoom).reshape(
        req.deltaBlock[0], req.deltaBlock[1]
    )

    yPadCounter = 0

    while x < req.deltaBlock[0] and y < req.deltaBlock[1]:
        cord = getNextPoint(
            x, req.visionScope, yPadCounter, sampledRoom, req.deltaBlock
        )
        yPad = int(floor(SCOPE_MULTIPLIER * req.visionScope * yPadCounter))

        if cords == [None, None]:
            break

        # get next cord
        nextX = int(floor(cord[0] + req.visionScope))
        nextY = int(floor(cord[1] + req.visionScope))

        nearestCurrent = findNearestPoint([nextX, nextY], sampledRoom)

        if nearestCurrent == [None, None]:
            break

        sampledRoom[nearestCurrent[0]][nearestCurrent[1]] = 2
        cords.append(
            [
                req.boundries[0] + nextX * req.blockSize,
                req.boundries[3] + nextY * req.blockSize,
            ]
        )

        # move current cord to the next one
        x = int(floor(nearestCurrent[0] + SCOPE_MULTIPLIER * req.visionScope))
        y = int(nearestCurrent[1])

        if not isRightPossible(x, y, sampledRoom):
            x = 0
            y += floor(SCOPE_MULTIPLIER * req.visionScope)
            yPadCounter += 1

        if y > len(sampledRoom[1]):
            break

    res.sampledRoom = list(sampledRoom.flatten())
    res.selectedLocations = list(np.array(cords).flatten())
    return res
