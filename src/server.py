#! /usr/bin/python

import sys, math, threading
from time import sleep

import rospy
from rcvm_core.srv import *
import gimbal, flight

# Global variables for flight control
puffin = None 

# Global variable for gimbal control.
z3 = None

# This lock will be used to ensure that only 1 RCVM service can be operating at a given time.
animation_lock = threading.Lock()

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
    Main RCVM initialization.
'''
if __name__ == "__main__":
    rospy.init_node('rcvm_server', argv=None, anonymous=True)
    rospy.loginfo('Initializing Matrice 100 RCVM server...')

    puffin = FlightControl()

    # Check activation
    rospy.loginfo('   Checking activation...')
    if puffin.activate():
        rospy.loginfo('      Activation successful.')
    else:
        rospy.logerr('      Activation failed!')
        sys.exit()

    # Check matrice version
    rospy.loginfo('   Checking drone version...')
    response = puffin.querryVersion()
    if response.version == flight.FIRMWARE_3_1_10 and response.hardware == flight.HARDWARE:
        rospy.loginfo('      Connected to Matrice 100, Firmware version 3.1.10')
    else:
        rospy.logerr('      Hardware or firmware incompatible!')
        sys.exit()

    # Request control. 
    #TODO In future, control should only requested when a RCVM service is called.
    rospy.loginfo('   Requesting control...')
    if self.requestControl():
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
