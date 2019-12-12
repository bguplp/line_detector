#!/usr/bin/env python

import rospy

import tf2_ros
import tf.transformations

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from actionlib_msgs.msg import GoalStatus

from line_detector.srv import NextPositionInLineService, ser_message, ser_messageResponse
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Twist, Quaternion
from tf2_geometry_msgs import PoseStamped


rospy.init_node('queue_end_detection')

def queue_end_detection_func(req):
    print("waiting for services")
    rospy.wait_for_service('line_detection_service')
    print("got line service")
    #rospy.wait_for_service('gazebo/set_model_state')
    #print("got gazebo service")

    line_srv = rospy.ServiceProxy('line_detection_service', NextPositionInLineService)
    #teleport_srv = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

    print("setting tf buffer")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    print("submitting request")
    result = line_srv(0.8, "ltr")
    print("got it!")
    print(result.next_position)

    print("converting to map frame")

    # Assuming the world is static, you could use this fix. Which will force the coordinates transformation to use the last data which there is on the frame, without respect to the time period.
    result.next_position.header.stamp = rospy.Time(0)

    input_pose = PoseStamped(result.next_position.header, result.next_position.pose)
    converted_pose = tfBuffer.transform(input_pose, 'map').pose
    converted_pose.position.z = 0 # ground the position

    # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,angle_temp[2]))

    print(converted_pose)

    #twist = Twist()
    #model_name = "armadillo2"

    #desired_state = ModelState(model_name, converted_pose, twist, 'map')
    #print("teleporting! That probably messes with the transform tree...")
    #teleport_srv(desired_state)

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(converted_pose.position.x,converted_pose.position.y,0)
    goal.target_pose.pose.orientation.x = converted_pose.orientation.x
    goal.target_pose.pose.orientation.y = converted_pose.orientation.y
    goal.target_pose.pose.orientation.z = converted_pose.orientation.z
    goal.target_pose.pose.orientation.w = converted_pose.orientation.w

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the end of the queue")
        # ser_messageResponse(True)
    else:
        rospy.loginfo("The robot failed to reach the end of the queue")
        # ser_messageResponse(False)

rospy.Service("/line_end_detection", ser_message, queue_end_detection_func)
rospy.loginfo("queue end detection service is waiting for request...")
rospy.spin()