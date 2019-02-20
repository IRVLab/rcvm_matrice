#! /usr/bin/python

import sys, math, threading
from time import sleep

import rospy
from rcvm_core.srv import *
import gimbal, flight

# Global variables for flight control
matrice = None 

# Global variable for gimbal control.
z3 = None

# This lock will be used to ensure that only 1 RCVM service can be operating at a given time.
animation_lock = threading.Lock()

'''
    Service handlers.
'''

def affirmative_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        z3.setMode(gimbal.REL_PITCH)
        z3.command(0, 30, 0)
        z3.command(0, -50, 0)
        z3.command(0, 50, 0)
        z3.command(0, -30, 0)
        
        return True


def attention_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        matrice.goToTarget(0,0,0,90)
        matrice.goToTarget(0,0,0,-90)
        matrice.goToTarget(0,0,0,90)
        matrice.goToTarget(0,0,0,-90)

        return True

def danger_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        matrice.goToTarget(2,0,0,0)
        z3.command(0,0,60)
        z3.command(0,0,-120)
        z3.command(0,0,120)
        z3.command(0,0,-120)
        z3.command(0,0,60)
        matrice.goToTarget(0,0,0,180)
        matrice.goToTarget(-5,0,0,0)
        
        return True

def follow_me_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        matrice.goToTarget(0,0,0, 30)
        z3.command(0,0,60)
        z3.command(0,0,-10)
        z3.command(0,0,10)
        z3.command(0,0,-10)
        z3.command(0,0, 10)
        matrice.goToTarget(0,0,0, 150)
        matrice.goToTarget(5,0,0, 0 )
        
        return True


def indicate_movement_handler(req):
    global animation_lock, z3, matrice

    movement_vector = req.direction
    x = movement_vector.x 
    y = movement_vector.y

    with animation_lock:
        if x == 1:
            matrice.goToTarget(2, 0, 0, 0)
            matrice.goToTarget(-2, 0, 0, 0)
            matrice.goToTarget(2, 0, 0, 0)
            matrice.goToTarget(-2, 0, 0, 0)
            matrice.goToTarget(2, 0, 0, 0)
            return True
        elif x == -1:
            matrice.goToTarget(-2, 0, 0, 0)
            matrice.goToTarget(2, 0, 0, 0)
            matrice.goToTarget(-2, 0, 0, 0)
            matrice.goToTarget(2, 0, 0, 0)
            matrice.goToTarget(-2, 0, 0, 0)
            return True
        else:
            if y == 1:
                matrice.goToTarget(0, 2, 0, 0)
                matrice.goToTarget(0, -2, 0, 0)
                matrice.goToTarget(0, 2, 0, 0)
                matrice.goToTarget(0, -2, 0, 0)
                matrice.goToTarget(0, 2, 0, 0)
                return True
            elif y == -1
                matrice.goToTarget(0, -2, 0, 0)
                matrice.goToTarget(0, 2, 0, 0)
                matrice.goToTarget(0, -2, 0, 0)
                matrice.goToTarget(0, 2, 0, 0)
                matrice.goToTarget(0, -2, 0, 0)
                return True
            else:
                return True


def indicate_object_handler(req):
    global animation_lock, z3, matrice

    q = req.relative_orientation
    rads = euler_from_quaternion([q.x, q.y, q.z, q.w])
    roll  = int(flight.RAD2DEG(rads[0]))
    pitch = int(flight.RAD2DEG(rads[1]))
    yaw   = int(flight.RAD2DEG(rads[2]))

    with animation_lock:
        matrice.goToTarget(0,0,0,yaw)
        z3.command(0,pitch,0)
        z3.command(0,-pitch,0)
        z3.command(0,0,-yaw)
        sleep(2)
        z3.command(0,pitch,yaw)
        z3.command(0,0)
        
        return True

def indicate_stay_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        matrice.goToTarget(3, 0, 0, 90)
        matrice.goToTarget(0, 3, 0, 90)
        matrice.goToTarget(-3, 0, 0, 90)
        matrice.goToTarget(0, -3, 0, 90)
        
        return True

def lost_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        return False

def malfunction_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        z3.command(0,-90,0)
        matrice.goToTarget(0,0,0,180)
        matrice.goToTarget(0,0,0,180)
        
        return True
        

def negative_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        z3.setMode(gimbal.RELL_YAW)
        z3.command(0,0,20)
        z3.command(0,0,-40)
        z3.command(0,0,40)
        z3.command(0,0,-40)
        z3.command(0,0,40)
        z3.command(0,0,-20)

        return True

def possibly_handler(req):
    global animation_lock, z3, matrice    
    with animation_lock:
        z3.command(20, 0, 0 )
        z3.command(-40, 0, 0)
        z3.command(40, 0, 0)
        z3.command(-20, 0, 0)
        
        return True

def repeat_last_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        matrice.goToTarget(0,0,0,30)
        z3.command(25, 0, 10)
        sleep(2)
        matrice.goToTarget(0,0,0,-30)
        z3.command(-25, 0, -10)
        
        return True

def report_battery_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        matrice.goToTarget(0,0,0,180)
        matrice.goToTarget(0,0,0,180)
        matrice.goToTarget(0,0,2)
        matrice.goToTarget(0,0,-2)
        matrice.goToTarget(0,0,0,180)
        matrice.goToTarget(0,0,0,180)
        
        return True

'''
    Main RCVM initialization.
'''
if __name__ == "__main__":
    rospy.init_node('rcvm_server', argv=None, anonymous=True)
    rospy.loginfo('Initializing Matrice 100 RCVM server...')

    matrice = flight.FlightControl()
    rospy.on_shutdown(matrice.land)

    # Check matrice version
    rospy.loginfo('   Checking drone version...')
    response = matrice.queryVersion()
    if response.version == flight.FIRMWARE_3_1_10 and response.hardware == flight.HARDWARE:
        rospy.loginfo('      Connected to Matrice 100, Firmware version 3.1.10')
    else:
        rospy.logerr('      Hardware or firmware incompatible!')
        sys.exit()

    # Request control. 
    #TODO In future, control should only requested when a RCVM service is called.
    rospy.loginfo('   Requesting control...')
    if matrice.requestControl():
        rospy.loginfo('      Control approved.')
    else:
        rospy.logerr('      Control denied!')
        sys.exit()

    # Take off aircraft
    # TODO In future, just check if the aircraft is airborn before each kineme, and use short range versions if it has not.
    rospy.loginfo('   Taking off!')
    if matrice.takeoff():
        rospy.loginfo('Takeoff successful!')
    else:
        rospy.logerr('Takeoff unsuccessful.')
        matrice.land()
        sys.exit()

    # Set up the local opsition reference.
    if matrice.setLocalPose():
	    rospy.loginfo('Set Local Pose Reference successfully.')
    else:
	    rospy.logerr('Failed to set local pose reference.')
        matrice.land()
        sys.exit()

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
