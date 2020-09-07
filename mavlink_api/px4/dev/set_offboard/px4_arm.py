import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool,SetMode

if __name__ == '__main__':

    armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    rospy.init_node("offboard_node")
    rate = rospy.Rate(20)

    #arm vehicle
    armService(True)




