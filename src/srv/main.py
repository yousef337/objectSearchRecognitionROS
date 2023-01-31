#!/usr/bin/env python
import rospy
from object_search.srv import (
    ObjectFinder,
    ObjectFinderRequest,
    ObjectFinderResponse,
)
from object_search.srv import LocationSampling, Rasterization, ObjectDetection
from yolo_object_detection.srv import YoloDetection, YoloDetectionResponse
import smach
from smach import Concurrence, CBState
from smach_ros import SimpleActionState, IntrospectionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image, PointCloud2
import numpy as np
from lasr_perception_server.srv import DetectImage
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from geometry_msgs.msg import Twist


servers = []
found = False


def prepareData(room, blockSize, visionScope):
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



def stopAllServers():
    global servers
    for i in servers:
        i.cancel_all_goals()


def objectDetectionCB(user_data):
    rospy.wait_for_service('lasr_perception_server/detect_objects_image')
    objectRecognitionService = rospy.ServiceProxy(
        'lasr_perception_server/detect_objects_image', DetectImage
    )

    while not rospy.is_shutdown():
        img_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        resp = objectRecognitionService(
            [img_msg], 'coco', 0.7, 0.3, [user_data.searchFor], 'yolo'
        ).detected_objects

        if user_data.searchFor in list(map(lambda f: f.name, resp)):
            global found
            found = True
            stopAllServers()

            return 'found'

    return 'notFound'


def moveControllerCB(user_data):

    if user_data.index + 1 >= len(user_data.cords):
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


def runClient(serverName, msgType, goal):
    global servers

    client = actionlib.SimpleActionClient(serverName, msgType)
    client.wait_for_server()
    client.send_goal(goal)
    servers.append(client)
    client.wait_for_result()


def movePerceptionAround(user_data):
    d = 180
    global found

    for i in range(360 // d):
        moveBaseGoal = MoveBaseGoal()
        moveBaseGoal.target_pose.header.stamp = rospy.get_rostime()
        moveBaseGoal.target_pose.header.frame_id = 'map'
        moveBaseGoal.target_pose.pose.position.x = user_data.cords[
            user_data.index
        ][0]
        moveBaseGoal.target_pose.pose.position.y = user_data.cords[
            user_data.index
        ][1]
        moveBaseGoal.target_pose.pose.orientation.z = d * i
        moveBaseGoal.target_pose.pose.orientation.w = 1

        if not found:
            runClient('move_base', MoveBaseAction, moveBaseGoal)

        playMotionGoal = PlayMotionGoal()
        playMotionGoal.motion_name = 'head_tour'

        if not found:
            runClient('play_motion', PlayMotionAction, playMotionGoal)

    user_data.index = user_data.index
    user_data.cords = user_data.cords
    return 'next'


def child_term_cb(outcome_map):
    if outcome_map['ObjectDetection'] == 'found':
        return True

    return False


def objectFinderCB(req):
    cords = prepareData(req.room, req.blockSize, req.visionScope)

    objectFinder = Concurrence(
        outcomes=['found', 'notFound'],
        default_outcome='notFound',
        child_termination_cb=child_term_cb,
        outcome_map={
            'found': {'ObjectDetection': 'found'},
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
            input_keys=['index', 'cords'], outcomes=['end', 'terminated']
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
                'MovePerceptionAround',
                CBState(
                    movePerceptionAround,
                    input_keys=['index', 'cords'],
                    output_keys=['index', 'cords'],
                    outcomes=['next'],
                ),
                transitions={'next': 'MoveController'},
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
                    'succeeded': 'MovePerceptionAround',
                    'aborted': 'MovePerceptionAround',
                    'preempted': 'terminated',
                },
            )

        smach.Concurrence.add('MoveParent', moveParent)

    sis = IntrospectionServer('ObjectSearch', objectFinder, '/SM_ROOT')
    sis.start()
    stateRes = objectFinder.execute()

    res = ObjectFinderResponse()
    res.found = True if stateRes == 'found' else False

    return res
