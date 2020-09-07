import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool,SetMode, SetMavFrameRequest, SetMavFrameResponse, SetMavFrame
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
import time
import math




if __name__ == '__main__':
   
    rospy.init_node("GiMav_Frame_Switch")
    rate = rospy.Rate(20)

    # for arming and changing mode
    armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    mav_frame_switch = rospy.ServiceProxy('/mavros/setpoint_position/mav_frame', SetMavFrame)
        

    #uint8 FRAME_GLOBAL=0
    #uint8 FRAME_LOCAL_NED=1
    #uint8 FRAME_MISSION=2
    #uint8 FRAME_GLOBAL_RELATIVE_ALT=3
    #uint8 FRAME_LOCAL_ENU=4
    #uint8 FRAME_GLOBAL_INT=5
    #uint8 FRAME_GLOBAL_RELATIVE_ALT_INT=6
    #uint8 FRAME_LOCAL_OFFSET_NED=7
    #uint8 FRAME_BODY_NED=8
    #uint8 FRAME_BODY_OFFSET_NED=9
    #uint8 FRAME_GLOBAL_TERRAIN_ALT=10
    #uint8 FRAME_GLOBAL_TERRAIN_ALT_INT=11

    frame = 9
    ret = mav_frame_switch(frame)

    if(ret.success):
        print "Mav Frame Set to :", frame
    else:
        print "Failed setting Mav Frame!"
