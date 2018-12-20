#! /usr/bin/python

import sys

import rospy
from dji_sdk.msg import Gimbal
from dji_sdk.srv import Activation
from rcvm_core.srv import *

if __name__ == "__main__":
    rospy.init_node('rcvm_server', argv=None, anonymous=True)
    rospy.loginfo('Initializing Matrice 100 RCVM server...')

    activate = rospy.ServiceProxy('dji_sdk/activation', Activation)

    response = activate()
    if response.result:
        rospy.loginfo('Activation successful.')
    else:
        rospy.logerr('Activation failed.')
        sys.exit()

else:
    pass