from mavros_msgs.msg import GlobalPositionTarget
import rospy
from mavros_msgs.srv import *
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import time
import math


# declare global variables
global_imu = Imu()
local_pose = PoseStamped()
body_position = PoseStamped()
local_position = PoseStamped()
global_position = PoseStamped()
current_pose = PoseStamped()
current_mode = String()


def position_add(pose1, pose2):
    result = PoseStamped()
    result.header.stamp = rospy.Time.now()
    result.pose.position.x = pose1.pose.position.x + pose2.pose.position.x
    result.pose.position.y = pose1.pose.position.y + pose2.pose.position.y
    result.pose.position.z = pose1.pose.position.z + pose2.pose.position.z
    return result

def heading_add(pose1, pose2):
    result = PoseStamped()
    result.header.stamp = rospy.Time.now()

    return result

def imu_callback(data):
    global global_imu
    global_imu = data


def pose_callback(data):
    global local_pose
    local_pose = data

def mode_callback(data):
    global current_mode
    current_mode = data

def body_position_callback(data):
    global body_position, current_pose
    body_position = data
    current_pose = position_add(current_pose, body_position)

def body_heading_callback(data):
    global body_heading, current_pose
    body_heading = data
    current_pose = position_add(current_pose, body_heading)
    current_pose = heading_add(current_pose, body_heading)


def local_position_callback(data):
    global local_position
    local_position = data

def global_position_callback(data):
    global global_position
    global_position = data


def relative_distance(cur_pose, target_pose):
    delta_x = math.fabs(cur_pose.pose.position.x - target_pose.pose.position.x)
    delta_y = math.fabs(cur_pose.pose.position.y - target_pose.pose.position.y)
    delta_z = math.fabs(cur_pose.pose.position.z - target_pose.pose.position.z)

    return delta_x + delta_y + delta_z


def set_pose(x=0, y=0, z=2, use_current_heading=True):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    if (use_current_heading):
        #print("Using current heading!")
        pose.pose.orientation = global_imu.orientation
    else:
        print("Not using current heading!")
        # modify this part

    return pose




if __name__ == '__main__':

    # for arming and changing mode
    armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)

    # for fetching current IMU data and local pose
    imu_sub = rospy.Subscriber("mavros/imu/data", Imu, imu_callback)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_callback)

    # for setting target position in local and global frame
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    global_gps_pub = rospy.Publisher('mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)

    # for receiving commands
    position_local_sub = rospy.Subscriber("gi/set_position/local", PoseStamped, local_position_callback)
    position_global_sub = rospy.Subscriber("gi/set_position/global", PoseStamped, global_position_callback)
    position_body_sub = rospy.Subscriber("gi/set_position/body", PoseStamped, body_position_callback)
    heading_body_sub = rospy.Subscriber("gi/set_position/body", PoseStamped, body_position_callback)
    mode_command_sub = rospy.Subscriber("gi/set_mode", String, mode_callback)

    rospy.init_node("px4_mavros_node")
    rate = rospy.Rate(20)

    print("System initializing ...")

    for i in range(100):
        local_pos_pub.publish(set_pose())
        rate.sleep()

    current_mode = String("OFFBOARD")
    print("System initialized! Waiting for commands!")

    while not rospy.is_shutdown():

        # keep the drone guided and armed
        if current_mode == String("OFFBOARD"):
            print("OFFBOARD MODE!")

            isModeChanged = flightModeService(custom_mode='OFFBOARD')
            if not isModeChanged:
                print("Cannot set offboard mode! Stopping service!")
                rospy.signal_shutdown("Shutting down!")
            isArmed = armService(True)
            if not isArmed:
                print("Cannot ARM! Stopping service!")
                rospy.signal_shutdown("Shutting down!")

            # add body relevant pose to initial body pose
            #current_pose = position_add(current_pose, body_position)
            local_pos_pub.publish(current_pose)


        elif current_mode == String("AUTO.LAND"):

            print("AUTO.LAND MODE!")
            isModeChanged = flightModeService(custom_mode='AUTO.LAND')
            if not isModeChanged:
                print("Cannot set AUTO.LAND mode!")


        elif current_mode == String("shabi"):

            print("shabi MODE!")


        else:

            print(current_mode, " MODE not supported!")



