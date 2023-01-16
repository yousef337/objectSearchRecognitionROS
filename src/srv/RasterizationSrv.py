#!/usr/bin/env python3
import rospy
import settings
from object_search.srv import (
    Rasterization,
    RasterizationRequest,
    RasterizationResponse,
)
from os import getcwd
from math import floor
import numpy as np
from geojson import load


def retrieveRoomBoundaries(room):
    featureFile = getcwd() + '/src/object_search/rooms/rooms.geojson'
    with open(featureFile) as f:
        features = load(f)['features']
        for feature in features:
            if feature['properties']['name'] == room:
                return feature['geometry']['coordinates']


# https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
def ccw(p1, p2, p3):
    return (p3[1] - p1[1]) * (p2[0] - p1[0]) > (p2[1] - p1[1]) * (
        p3[0] - p1[0]
    )


def intersect(l1, l2):
    return ccw(l1[0], l2[0], l2[1]) != ccw(l1[1], l2[0], l2[1]) and ccw(
        l1[0], l1[1], l2[0]
    ) != ccw(l1[0], l1[1], l2[1])


def isInsideRoom(point, polygonCords):
    orthLine = (
        (point[0], point[1]),
        (point[0], point[1] + settings.POLYGON_CORNOR_FRAME),
    )
    inside = False
    for i in range(len(polygonCords)):
        if intersect(
            orthLine,
            ((polygonCords[i]), (polygonCords[(i + 1) % len(polygonCords)])),
        ):
            inside = not inside

    return inside


def rasterizationCB(req):
    roomBoundaries = retrieveRoomBoundaries(req.name)

    boundries = [None, None, None, None]   # x left, y top, x right, y bottom

    for i in roomBoundaries:
        # x left
        if not boundries[0] or i[0] < boundries[0]:
            boundries[0] = i[0]

        # y top
        if not boundries[1] or i[1] > boundries[1]:
            boundries[1] = i[1]

        # x right
        if not boundries[2] or i[0] > boundries[2]:
            boundries[2] = i[0]

        # y bottom
        if not boundries[3] or i[1] < boundries[3]:
            boundries[3] = i[1]

    # rasterize
    listXSize = int(abs(floor((boundries[2] - boundries[0]) / req.blockSize)))
    listYSize = int(abs(floor((boundries[1] - boundries[3]) / req.blockSize)))
    rasterisedRoom = np.ones([listXSize, listYSize], int)

    # set to 0 where it is outside the room
    for i in range(listYSize):
        for j in range(listXSize):
            if not isInsideRoom(
                [
                    boundries[0] + j * req.blockSize,
                    boundries[3] + i * req.blockSize,
                ],
                roomBoundaries,
            ):
                rasterisedRoom[j][i] = 0
            else:
                rasterisedRoom[j][i] = 1

    res = RasterizationResponse()
    res.deltaBlock = [listXSize, listYSize]
    res.boundries = boundries
    res.rasterizedRoom = list(rasterisedRoom.flatten())

    return res
