import rospy
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool,SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
import time
import math
rospy.init_node("commander_node")
rate = rospy.Rate(20)
local_setposition_pub = rospy.Publisher('gi/set_pose/position', PoseStamped, queue_size=10)
local_setorientation_pub = rospy.Publisher('gi/set_pose/orientation', Float32, queue_size=10)
def set_pose(x=0, y=0, z=2):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    return pose
local_setposition_pub.publish(set_pose(0,0,2))
local_setorientation_pub.publish(90)
