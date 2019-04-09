#! /usr/bin/python

import sys, math, threading
from time import sleep

import rospy, tf
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
        z3.reset()
        
        return True

# Rotate twice with a pause in between.
def attention_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        z3.setMode(gimbal.REL_ALL)
        z3.command(60,0,0)
        z3.command(-120,0,0)
        z3.command(120,0,0)
        z3.command(-120,0,0)
        z3.command(120,0,0)
        z3.command(-60,0,0)
        z3.reset()

# Move forward, (gimbal) look around, turn around, move away.
def danger_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        z3.setMode(gimbal.REL_YAW)
        z3.command(0,0,45)
        sleep(1)
        z3.command(0,0,-90)
        sleep(1)
        z3.command(0,0,90)
        sleep(1)
        z3.command(0,0,-45)

        z3.command(0,0,15)
        z3.command(0,0,-30)
        z3.command(0,0,30)
        z3.command(0,0,-30)
        z3.command(0,0,30)
        z3.command(0,0,-30)
        z3.command(0,0,30)
        z3.command(0,0,-15)

        z3.reset()

        return True

# Turn slight, beckon (gimbal), turn fully around, move away.
def follow_me_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        z3.setMode(gimbal.REL_ALL)
        z3.command(0,0,100)
        sleep(1)
        z3.command(0,0,-60)
        z3.command(25,0, 40)
        z3.command(-25,0,-40)
        z3.command(25,0, 100)
        sleep(2)

        z3.reset()
        
        return True

def indicate_movement_handler(req):
    global animation_lock, z3, matrice

    movement_vector = req.direction
    x = movement_vector.x 
    y = movement_vector.y

    pitch_amount = 15
    yaw_amount = 50

    z3.setMode(gimbal.REL_ALL)

    with animation_lock:
        z3.command(0, (x*pitch_amount), (y*yaw_amount))
        sleep(1)
        z3.command(25,0,0)
        z3.command(-25,0,0)
        z3.command(0, -(x*pitch_amount), -(y*yaw_amount))
        z3.command(0, (x*pitch_amount), (y*yaw_amount))
        sleep(1)
        z3.command(25,0,0)
        z3.command(-25,0,0)
        z3.command(0, -(x*pitch_amount), -(y*yaw_amount))
        z3.reset()

        return True


def indicate_object_handler(req):
    global animation_lock, z3, matrice

    q = req.relative_orientation
    rads = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    roll  = int(flight.RAD2DEG(rads[0]))
    pitch = int(flight.RAD2DEG(rads[1]))
    yaw   = int(flight.RAD2DEG(rads[2]))

    with animation_lock:
        z3.setMode(gimbal.ABS_ALL)
        z3.command(0,-pitch,yaw)
        sleep(1)
        z3.command(0,pitch,z3.current_position.z)
        z3.command(0,-pitch,z3.current_position.z)
        sleep(2)
        z3.reset()
        
        return True

def indicate_stay_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        z3.setMode(gimbal.REL_YAW)

        z3.command(0,0, 105,t=2)
        z3.command(0,0,-205,t=3)
        z3.command(0,0, 205,t=3)
        z3.command(0,0,-105,t=2)
        z3.reset()
        
        return True

def lost_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        z3.setMode(gimbal.REL_ALL)
        z3.command(0,0,90)        
        z3.command(15,0,0)
        z3.command(0,10,0)
        z3.command(-15,0,0)
        z3.command(0,-10,0)
        z3.command(-15,0,0)
        z3.command(0,-10,0)
        z3.command(15,0,0)
        z3.command(0,10,0)
        sleep(1)

        z3.command(0,0,-180)
        z3.command(15,0,0)
        z3.command(0,10,0)
        z3.command(-15,0,0)
        z3.command(0,-10,0)
        z3.command(-15,0,0)
        z3.command(0,-10,0)
        z3.command(15,0,0)
        z3.command(0,10,0)
        sleep(1)

        z3.command(0,0,90)

        z3.command(0,0,15)
        z3.command(0,0,-30)
        z3.command(0,0,30)
        z3.command(0,0,-15)
        z3.reset()
        return True

def malfunction_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:            
        z3.setMode(gimbal.REL_ALL)
        z3.command(0,-90,0)
        z3.command(0,0, 180, t=5)
        sleep(2)
        z3.command(0,0, -180, t=5)
        z3.reset()

        return True
        

def negative_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:
        z3.setMode(gimbal.REL_YAW)
        z3.command(0,0,20)
        z3.command(0,0,-40)
        z3.command(0,0,40)
        z3.command(0,0,-40)
        z3.command(0,0,40)
        z3.command(0,0,-20)
        z3.reset()

        return True

def repeat_last_handler(req):
    global animation_lock, z3, matrice
    with animation_lock:

        z3.reset()
        z3.setMode(gimbal.REL_ALL)
        z3.command(45, 0, 40)
        sleep(4)

        z3.reset()
        
        return True

def report_battery_handler(req):
    return False
#     global animation_lock, z3, matrice
#     with animation_lock:
#         matrice.goToBodyTarget(yaw=1.0, duration=5)
#         matrice.goToBodyTarget(dz=1.0, duration=5)
#         matrice.goToBodyTarget(dz=-1.0, duration=5)
#         matrice.goToBodyTarget(yaw=-1.0, duration=5)
               
#         return True

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
#    rospy.loginfo('   Taking off!')
#    if matrice.takeoff():
#        rospy.loginfo('Takeoff successful!')
#    else:
#        rospy.logerr('Takeoff unsuccessful.')
#        matrice.land()
#        sys.exit()

    # Set up the local opsition reference.
#    if matrice.setLocalPose():
#	rospy.loginfo('Set Local Pose Reference successfully.')
#    else:
#	rospy.logerr('Failed to set local pose reference.')
#        matrice.land()
#        sys.exit()

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
