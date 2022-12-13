#!/usr/bin/env python3
import rospy
from object_search.srv import (
    ObjectFinder,
    ObjectFinderRequest,
    ObjectFinderResponse,
)
from object_search.srv import LocationSampling, Rasterization, ObjectDetection
import smach
from smach import Concurrence, CBState
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np


def prepareData(room: str, blockSize: float, visionScope: float):
    rasterRes = rospy.ServiceProxy('Rasterization', Rasterization)(
        room, blockSize
    )
    sampleRes = rospy.ServiceProxy('LocationSampling', LocationSampling)(
        rasterRes.deltaBlock,
        rasterRes.rasterizedRoom,
        rasterRes.boundries,
        blockSize,
        visionScope,
    )
    return list(
        np.array(sampleRes.selectedLocations).reshape(
            len(sampleRes.selectedLocations) // 2, 2
        )
    )


def objectDetectionCB(user_data):
    rospy.wait_for_service('objectDetection')
    objectRecognitionService = rospy.ServiceProxy(
        'objectDetection', ObjectDetection
    )

    while not rospy.is_shutdown():
        if objectRecognitionService(user_data.searchFor).found:
            return 'found'

    return 'notFound'


def moveControllerCB(user_data):
    if user_data.index >= len(user_data.cords):
        return 'end'

    moveBaseGoal = MoveBaseGoal()
    moveBaseGoal.target_pose.header.stamp = rospy.get_rostime()
    moveBaseGoal.target_pose.header.frame_id = 'map'
    moveBaseGoal.target_pose.pose.position.x = user_data.cords[
        user_data.index + 1
    ][0]
    moveBaseGoal.target_pose.pose.position.y = user_data.cords[
        user_data.index + 1
    ][1]
    moveBaseGoal.target_pose.pose.orientation.z = 1

    user_data.index = user_data.index + 1
    user_data.cords = user_data.cords

    user_data.target_pose = moveBaseGoal.target_pose

    return 'next'


def objectFinderCB(req: ObjectFinderRequest) -> ObjectFinderResponse:
    cords = prepareData(req.room, req.blockSize, req.visionScope)

    objectFinder = Concurrence(
        outcomes=['found', 'notFound'],
        default_outcome='notFound',
        outcome_map={
            'found': {'ObjectDetection': 'found'},
            'notFound': {'MoveParent': 'end'},
        },
    )

    objectFinder.userdata.index = -1
    objectFinder.userdata.cords = cords
    objectFinder.userdata.searchFor = req.name

    with objectFinder:
        smach.Concurrence.add(
            'ObjectDetection',
            CBState(
                objectDetectionCB,
                input_keys=['searchFor'],
                outcomes=['found', 'notFound'],
            ),
        )

        moveParent = smach.StateMachine(
            input_keys=['index', 'cords'], outcomes=['end']
        )

        with moveParent:
            smach.StateMachine.add(
                'MoveController',
                CBState(
                    moveControllerCB,
                    input_keys=['index', 'cords'],
                    output_keys=['index', 'cords', 'target_pose'],
                    outcomes=['next', 'end'],
                ),
                transitions={'next': 'MoveAction', 'end': 'end'},
            )

            smach.StateMachine.add(
                'MoveAction',
                SimpleActionState(
                    'move_base',
                    MoveBaseAction,
                    input_keys=['target_pose', 'cords', 'index'],
                    goal_slots=['target_pose'],
                    output_keys=['index', 'cords'],
                ),
                transitions={
                    'succeeded': 'MoveController',
                    'aborted': 'MoveController',
                    'preempted': 'MoveController',
                },
            )

        smach.Concurrence.add('MoveParent', moveParent)

    stateRes = objectFinder.execute()

    res = ObjectFinderResponse()
    res.found = True if stateRes == 'found' else False

    return res
