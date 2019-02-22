#! /usr/bin/python

import math, threading
import sys, select, termios, tty
from time import sleep

import rospy, tf
import gimbal, flight

# Global variables for flight control
matrice = None 

# Global variable for gimbal control.
z3 = None

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def forward_back():
    global matrice

    matrice.goToBodyTarget(pitch=0.25, duration=1, post_sleep=5)
    matrice.goToBodyTarget(pitch=-0.25, duration=1, post_sleep=5)


def left_right():
    global matrice

    matrice.goToBodyTarget(roll=0.25, duration=1, post_sleep=5)
    matrice.goToBodyTarget(roll=-0.25, duration=1, post_sleep=5)

def up_down():
    global matrice

    matrice.goToBodyTarget(dz=1.0, duration=1, post_sleep=5)
    matrice.goToBodyTarget(dz=-1.0, duration=1, post_sleep=5)


def rotate():
    global matrice

    matrice.goToBodyTarget(yaw=1.0, duration=1, post_sleep=5)
    matrice.goToBodyTarget(yaw=-1.0, duration=1, post_sleep=5)
'''
    Main RCVM initialization.
'''
if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('rcvm_server', argv=None, anonymous=True)

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

    print ('[1] Foreward-backward')
    print ('[2] Left-Right')
    print ('[3] Up-Down')
    print ('[4] Rotate')
    print ('[q] Quit')

    # Initiate Gimbal control structure.
    z3 = gimbal.GimbalControl()
    try:
        # Spin forever to avoid early shutdown.
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            key = getKey()
            if key == '1':
                forward_back()
            elif key == '2':
                left_right()
            elif key == '3':
                up_down()
            elif key == '4':
                rotate()
            elif key == 'q':
                break
            else:
                rate.sleep()

    except Exception as e:
        print(e)

    finally:
        matrice.land()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
else:
    pass
