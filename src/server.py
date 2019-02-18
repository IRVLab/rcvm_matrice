#! /usr/bin/python

import sys, math, threading
from time import sleep

import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import PointStamped, QuaternionStamped, Vector3Stamped, Transform
from sensor_msgs.msg import NavSatFix, Joy

from dji_sdk.srv import Activation, SDKControlAuthority, QueryDroneVersion, DroneTaskControl
from dji_sdk.srv import ActivationRequest, SDKControlAuthorityRequest, QueryDroneVersionRequest, DroneTaskControlRequest
from dji_sdk.srv import ActivationResponse, SDKControlAuthorityResponse, QueryDroneVersionResponse, DroneTaskControlResponse

from rcvm_core.srv import *

import gimbal


# Firmware version definitions.
FIRMWARE_3_1_10 = 50399744
HARDWARE = 'M100'

# Task definitions
TASK_GOHOME = 1
TASK_TAKEOFF = 4
TASK_LAND = 6

# Publisher definitions
joy_cmd = None

# Target position lists
target_x_offset = list()
target_y_offset = list()
target_z_offset = list()
target_yaw_offset = list()

# Error threshold for local position measurements (changes based on hardware/payload/wind conditions)
position_threshold = 0.1

# Global variables for message data
flight_status = None
local_position = None
gps_fix = None
gps_health = None
interaction_distance = None
gaze_tf = None

# Global variable for gimbal control.
z3 = None

# This lock will be used to ensure that only 1 RCVM service can be operating at a given time.
animation_lock = threading.Lock()
sent = False

'''
    Subscriber callbacks. Pretty much all just assinging the message to a global variable, with any necessary preprocessing.
'''
def flight_status_cb(msg):
    global flight_status
    flight_status = msg

def pos_cb(msg):
    global local_position, sent
    local_position = msg.point

    if len(target_x_offset) > 0:
        if reachedTargetPosition():
            sent = False
            removeFirstTarget()
        else:
            if not sent:
                sent = True
                msg = Joy()
                msg.axes.append(target_x_offset - local_position.x)
                msg.axes.append(target_y_offset - local_position.y)
                msg.axes.append(target_z_offset)
                msg.axes.append(target_yaw_offset)

                joy_cmd.publish(msg)

def gps_fix_cb(msg):
    global gps_fix
    gps_fix = msg

def gps_health_cb(msg):
    global gps_health
    gps_health = msg

def distance_cb(msg):
    global interaction_distance
    interaction_distance = msg.data

def gaze_cb(msg):
    global gaze_tf
    gaze_tf = msg

'''
    Service handlers.
'''

def affirmative_handler(req):
    global z3

    z3.setMode(gimbal.REL_PITCH)
    z3.command(0, 30, 0)
    z3.command(0, -60, 0)
    z3.command(0, 60, 0)
    z3.command(0, -30, 0)
     
    return True


def attention_handler(req):
    global animation_lock
    with animation_lock:
        if shortRange():
            #We're in short range, so we need to control the camera gimbal to do our kineme.
            pass
        else:
            #We are at long range, so we need to control the Matrice itself.
            pass

def danger_handler(req):
    global animation_lock
    with animation_lock:
        if shortRange():
            #We're in short range, so we need to control the camera gimbal to do our kineme.
            pass
        else:
            #We are at long range, so we need to control the Matrice itself.
            pass
def follow_me_handler(req):
    global animation_lock
    with animation_lock:
        if shortRange():
            #We're in short range, so we need to control the camera gimbal to do our kineme.
            pass
        else:
            #We are at long range, so we need to control the Matrice itself.
            pass

def indicate_movement_handler(req):
    global animation_lock
    with animation_lock:
        if shortRange():
            #We're in short range, so we need to control the camera gimbal to do our kineme.
            pass
        else:
            #We are at long range, so we need to control the Matrice itself.
            pass

def indicate_object_handler(req):
    global animation_lock
    with animation_lock:
        if shortRange():
            #We're in short range, so we need to control the camera gimbal to do our kineme.
            pass
        else:
            #We are at long range, so we need to control the Matrice itself.
            pass

def indicate_stay_handler(req):
    global animation_lock
    with animation_lock:
        if shortRange():
            #We're in short range, so we need to control the camera gimbal to do our kineme.
            pass
        else:
            #We are at long range, so we need to control the Matrice itself.
            pass

def lost_handler(req):
    global animation_lock
    with animation_lock:
        if shortRange():
            #We're in short range, so we need to control the camera gimbal to do our kineme.
            pass
        else:
            #We are at long range, so we need to control the Matrice itself.
            pass

def malfunction_handler(req):
    global animation_lock
    with animation_lock:
        if shortRange():
            #We're in short range, so we need to control the camera gimbal to do our kineme.
            pass
        else:
            #We are at long range, so we need to control the Matrice itself.
            pass

def negative_handler(req):
    global animation_lock, z3
    with animation_lock:
        if shortRange():
            z3.setMode(gimbal.RELL_YAW)
            z3.command(0,0,20)
            z3.command(0,0,-40)
            z3.command(0,0,40)
            z3.command(0,0,-40)
            z3.command(0,0,40)
            z3.command(0,0,-20)

            return True
        else:
            #We are at long range, so we need to control the Matrice itself.

            # Here we're just going to add a series of target positions to the lists and then let the position handler deal with it.
            # We can control yaw on the aircraft.
            addTargetPosition(0,0,0,15)
            addTargetPosition(0,0,0,-15)
            addTargetPosition(0,0,0,15)
            addTargetPosition(0,0,0,-15)
            addTargetPosition(0,0,0,15)
            addTargetPosition(0,0,0,-15)

            return True

def possibly_handler(req):
    global animation_lock
    with animation_lock:
        if shortRange():
            #We're in short range, so we need to control the camera gimbal to do our kineme.
            msg = Gimbal()
            msg.mode = ABS_ROLL
            msg.yaw = 0
            msg.pitch = 0 

            msg.roll = DEG2RAD(25)
            gimbal_angle_cmd.publish(msg)
            sleep(0.1)

            msg.roll = DEG2RAD(-25)
            gimbal_angle_cmd.publish(msg)
            sleep(0.1)

            msg.roll = DEG2RAD(25)
            gimbal_angle_cmd.publish(msg)
            sleep(0.1)
            
            msg.roll = DEG2RAD(-25)
            gimbal_angle_cmd.publish(msg)
            sleep(0.1)

            return True
        else:
            #We are at long range, so we need to control the Matrice itself.
            
            # Here we're just going to add a series of target positions to the lists and then let the position handler deal with it.
            # This is series of movements left and right, attempting to get some roll going.
            addTargetPosition(0,1,0,0)
            addTargetPosition(0,-1,0,0)
            addTargetPosition(0,1,0,0)
            addTargetPosition(0,-1,0,0)
            addTargetPosition(0,1,0,0)
            addTargetPosition(0,-1,0,0)
            
            return True

def repeat_last_handler(req):
    global animation_lock
    with animation_lock:
        if shortRange():
            #We're in short range, so we need to control the camera gimbal to do our kineme.
            pass
        else:
            #We are at long range, so we need to control the Matrice itself.
            pass

def report_battery_handler(req):
    global animation_lock
    with animation_lock:
        if shortRange():
            #We're in short range, so we need to control the camera gimbal to do our kineme.
            pass
        else:
            #We are at long range, so we need to control the Matrice itself.
            pass

'''
    Utility functions and such
'''

# Checks if the current interaction distance is 10 meters or less, which is
# considered short range communication. If the distance is greater, we're
# doing long range communication.
def shortRange():
    global interaction_distance
    return interaction_distance <= 10

def addTargetPosition(x,y,z,yaw):
    global target_x_offset, target_y_offset, target_z_offset, target_yaw_offset

    target_x_offset.append(x)
    target_y_offset.append(y)
    target_z_offset.append(z)
    target_yaw_offset.append(yaw)

def reachedTargetPosition():
    global target_x_offset, target_y_offset, target_z_offset, target_yaw_offset
    global local_position, position_threshold

    return (abs(target_x_offset - local_position.x) < position_threshold) and (abs(target_y_offset - local_position.y) < position_threshold) \
    and (local_position.z > (target_z_offset - position_threshold)) and (local_position.z < (target_z_offset + position_threshold))

def removeFirstTarget():
    global target_x_offset, target_y_offset, target_z_offset, target_yaw_offset

    target_x_offset.append(0)
    target_y_offset.append(0)
    target_z_offset.append(0)
    target_yaw_offset.append(0)

'''
    Main RCVM initialization.
'''
if __name__ == "__main__":
    rospy.init_node('rcvm_server', argv=None, anonymous=True)
    rospy.loginfo('Initializing Matrice 100 RCVM server...')

    # Services
    activate = rospy.ServiceProxy('dji_sdk/activation', Activation)
    ctrl_auth = rospy.ServiceProxy('dji_sdk/sdk_control_authority', SDKControlAuthority)
    query_ver = rospy.ServiceProxy('dji_sdk/query_drone_version', QueryDroneVersion)
    drone_task = rospy.ServiceProxy('dji_sdk/drone_task_control', DroneTaskControl)

    # Publishers
    joy_cmd = rospy.Publisher('/dji_sdk/flight_control_setpoint_ENUposition_yaw', Joy, queue_size=10)

    # Subscribers
    rospy.Subscriber('/dji_sdk/flight_status', UInt8, flight_status_cb, queue_size=10)
    rospy.Subscriber('/dji_sdk/local_position', PointStamped, pos_cb, queue_size=10)
    rospy.Subscriber('/dji_sdk/gps_position', NavSatFix, gps_fix_cb, queue_size=10)
    rospy.Subscriber('/dji_sdk/gps_health', UInt8, gps_health_cb, queue_size=10)

    # RCVM Subscribers
    rospy.Subscriber('/rcvm/interaction_distance', UInt8, distance_cb, queue_size=10)
    rospy.Subscriber('/rcvm/gaze_direction', Transform, gaze_cb, queue_size=10)

    # Check activation
    rospy.loginfo('   Checking activation...')
    response = activate()
    if response.result:
        rospy.loginfo('      Activation successful.')
    else:
        rospy.logerr('      Activation failed!')
        sys.exit()

    # Check matrice version
    rospy.loginfo('   Checking drone version...')
    response = query_ver()
    if response.version == FIRMWARE_3_1_10 and response.hardware == HARDWARE:
        rospy.loginfo('      Connected to Matrice 100, Firmware version 3.1.10')
    else:
        rospy.logerr('      Hardware or firmware incompatible!')
        sys.exit()

    # Request control. 
    #TODO In future, control should only requested when a RCVM service is called.
    rospy.loginfo('   Requesting control...')
    response = ctrl_auth(1)
    if response.result:
        rospy.loginfo('      Control approved.')
    else:
        rospy.logerr('      Control denied!')
        sys.exit()

    # Take off aircraft
    # TODO In future, just check if the aircraft is airborn before each kineme, and use short range versions if it has not.
    rospy.loginfo('   Taking off!')
    response = drone_task(TASK_TAKEOFF)
    if response.result:
        rospy.loginfo('Takeoff successful!')
    else:
        rospy.logerr('Takeoff unsuccessful.')

    rospy.loginfo('   Advertising services...')

    # Initiate Gimbal control structure.
    z3 = gimbal.GimbalControl()

    #With the aircraft version and activation confirmed, we can advertise our services.
    rospy.Service('/rcvm/affirmative', Affirmative, affirmative_handler)
    rospy.Service('/rcvm/attention', Attention, attention_handler)
    rospy.Service('/rcvm/danger', Danger, danger_handler)
    rospy.Service('/rcvm/follow_me', FollowMe, follow_me_handler)
    rospy.Service('/rcvm/indicate_movement', IndicateMovement, indicate_movement_handler)
    rospy.Service('/rcvm/indicate_object', IndicateObject, indicate_object_handler)
    rospy.Service('/rcvm/indicate_stay', IndicateStay, indicate_stay_handler)
    rospy.Service('/rcvm/lost', Lost, lost_handler)
    rospy.Service('/rcvm/malfunction', Malfunction, malfunction_handler)
    rospy.Service('/rcvm/negative', Negative, negative_handler)
    rospy.Service('/rcvm/possibly', Possibly, possibly_handler)
    rospy.Service('/rcvm/repeat_last', RepeatLast, repeat_last_handler)
    rospy.Service('/rcvm/report_battery', ReportBattery, report_battery_handler)

    rospy.loginfo('      Service advertising completed...')
    rospy.loginfo('RCVM server ready for business!')
    rospy.loginfo('Spinning forever until a service request is recieved.')
    

    # Spin forever to avoid early shutdown.
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rate.sleep()
        
else:
    pass
