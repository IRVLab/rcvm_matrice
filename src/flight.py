#! /usr/bin/python

import sys, math, threading
from time import sleep

import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import UInt8
from geometry_msgs.msg import PointStamped, QuaternionStamped, Vector3Stamped, Transform
from sensor_msgs.msg import NavSatFix, Joy

from dji_sdk.msg import FlightAnomaly
from dji_sdk.srv import Activation, SDKControlAuthority, QueryDroneVersion, DroneTaskControl, SetLocalPosRef
from dji_sdk.srv import ActivationRequest, SDKControlAuthorityRequest, QueryDroneVersionRequest, DroneTaskControlRequest, SetLocalPosRefRequest
from dji_sdk.srv import ActivationResponse, SDKControlAuthorityResponse, QueryDroneVersionResponse, DroneTaskControlResponse, SetLocalPosRefResponse

# Firmware version definitions.
FIRMWARE_3_1_10 = 50399744
HARDWARE = 'M100'

# Flight status constants
STATUS_ON_GROUND        = 1
STATUS_TAKINGOFF        = 2
STATUS_IN_AIR           = 3
STATUS_LANDING          = 4
STATUS_FINISHED_LANDING = 5

# Flight anomoly constants
IMPACT_IN_AIR              = 1
RANDOM_FLY                 = 2
VERTICAL_CONTROL_FAIL      = 4
HORIZONTAL_CONTROL_FAIL    = 8
YAW_CONTROL_FAIL           = 16
AIRCRAFT_IS_FALLING        = 32
STRONG_WIND_LEVEL1         = 64
STRONG_WIND_LEVEL2         = 128
COMPASS_INSTALLATION_ERROR = 256
IMU_INSTALLATION_ERROR     = 512
ESC_TEMPERATURE_HIGH       = 1024
ESC_DISCONNECTED           = 2048
GPS_YAW_ERROR              = 4096

# Task definitions
TASK_GOHOME     = 1
TASK_TAKEOFF    = 4
TASK_LAND       = 6

# degree to radians and radian to degrees functions.
def DEG2RAD(deg):
    return deg * (math.pi/180)

def RAD2DEG(rad):
    return rad * (180/math.pi)

class FlightControl(object):
    def __init__(self):
        # Error threshold for local position measurements (changes based on hardware/payload/wind conditions)
        self.position_threshold = 0.1
        self.yaw_threshold = 1

        # Flight control publisher
        self.control = rospy.Publisher('/dji_sdk/flight_control_setpoint_ENUposition_yaw', Joy, queue_size=10)


        # Flight status and location info.
        self.flight_status = None
        self.flight_anomaly = None
        self.local_position = None
        self.attitude = None
        self.gps_fix = None
        self.gps_health = None

        # Subscribers
        rospy.Subscriber('/dji_sdk/flight_status', UInt8, self.flight_status_cb, queue_size=10)
        rospy.Subscriber('/dji_sdk/flight', FlightAnomaly, self.anomaly_cb, queue_size=10)
        rospy.Subscriber('/dji_sdk/local_position', PointStamped, self.pos_cb, queue_size=10)
        rospy.Subscriber('/dji_sdk/attitude', QuaternionStamped, self.att_cb, queue_size=10)
        rospy.Subscriber('/dji_sdk/gps_position', NavSatFix, self.gps_fix_cb, queue_size=10)
        rospy.Subscriber('/dji_sdk/gps_health', UInt8, self.gps_health_cb, queue_size=10)

        # Services
        self.activ = rospy.ServiceProxy('dji_sdk/activation', Activation)
        self.ctrl_auth = rospy.ServiceProxy('dji_sdk/sdk_control_authority', SDKControlAuthority)
        self.query_ver = rospy.ServiceProxy('dji_sdk/query_drone_version', QueryDroneVersion)
        self.drone_task = rospy.ServiceProxy('dji_sdk/drone_task_control', DroneTaskControl)
        self.set_pose_ref = rospy.ServiceProxy('dji_sdk/set_local_pos_ref', SetLocalPosRef)

    def goToTarget(self, x=0,y=0,z=0,yaw=0):
        msg = Joy()
        msg.axes.append(x) #- self.local_position.x)
        msg.axes.append(y)# - self.local_position.y)
        msg.axes.append(z)
        msg.axes.append(yaw)

        self.control.publish(msg)
        sleep(10)
    #    while not self.reachedPosition(x,y,z):
     #       sleep(1)

    # Checks whether or not you have reached the target local postion.
    def reachedPosition(self, x, y, z):
        return (abs(x - self.local_position.x) < self.position_threshold) and (abs(y - self.local_position.y) < self.position_threshold) \
        and (self.local_position.z > (z - self.position_threshold)) and (self.local_position.z < (z + position_threshold))

    # Checks whether or not you have reached the target yaw.
    def reachedYaw(self, yaw):
        return (abs(yaw - self.attitude[2]) <self.yaw_threshold)
    

    # Make sure the drone has been activated.
    def activate(self):
        rospy.wait_for_service('dji_sdk/activation')
        resp = self.activ()
        return resp.result

    # Check the hardware and firmware versions.
    def queryVersion(self):
        rospy.wait_for_service('dji_sdk/query_drone_version')
        return self.query_ver()

    # Request that flight control be turned over to the SDK.
    def requestControl(self):
        rospy.wait_for_service('dji_sdk/sdk_control_authority')
        resp = self.ctrl_auth(1)
        return resp.result

    # Do an arbitrary task.
    def doTask(self, task):
        rospy.wait_for_service('dji_sdk/drone_task_control')
        resp = self.drone_task(task)
        return resp.result

    # Take off with the drone.
    def takeoff(self):
        return self.doTask(TASK_TAKEOFF)

    # Land the drone.
    def land(self):
        return self.doTask(TASK_LAND)

    # Go to the home GPS location.
    def goHome(self):
        return self.doTask(TASK_GOHOME)

    # Do a monitored takeoff.
    def monitoredTakeoff(self):
        raise NotImplementedError
        
    #Set the local pose reference.
    def setLocalPose(self):
            resp = self.set_pose_ref()
            return resp.result


    '''
        Subscriber callbacks. Pretty much all just assinging the message to a global variable, with any necessary preprocessing.
    '''
    def flight_status_cb(self, msg):
        self.flight_status = msg.data

    def anomaly_cb(self, msg):
        self.flight_anomaly = msg.data

    def pos_cb(self, msg):
        self.local_position = msg.point

    def att_cb(self, msg):
        euler = euler_from_quaternion([msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w])
        self.attitude = (int(RAD2DEG(euler[0])), int(RAD2DEG(euler[1])), int(RAD2DEG(euler[2])))

    def gps_fix_cb(self, msg):
        self.gps_fix = msg

    def gps_health_cb(self, msg):
        self.gps_health = msg


        
