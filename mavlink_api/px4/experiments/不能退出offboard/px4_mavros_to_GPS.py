from mavros_msgs.msg import GlobalPositionTarget
import rospy
from mavros_msgs.srv import *
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
import time
import math

#global variables
global_imu = Imu()
global_gps = NavSatFix()


def imu_callback(data):
    global global_imu
    global_imu = data


def gps_callback(data):
    global global_gps
    global_gps = data


def set_pose(x=0, y=0, z=2, use_current_heading=True):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    print("target position set to: ", x, y, z)
    if (use_current_heading):
        print("using current heading!")
        pose.pose.orientation = global_imu.orientation
    else:
        print("Not using current heading!")
        # modify this part

    return pose

def set_gps(lat, long, alt):
    gps = GlobalPositionTarget()
    gps.header.stamp = rospy.Time.now()

    gps.latitude = float(lat)
    gps.longitude = float(long)
    gps.altitude = float(alt)

    print("GPS sent to: ", gps.latitude, gps.longitude, gps.altitude)
    return gps


def distance(gps1, gps2):
    delta_lat = math.fabs(gps1.latitude-gps2.latitude)
    delta_long = math.fabs(gps1.longitude - gps2.longitude)
    delta_alt = math.fabs(gps1.altitude - gps2.altitude)

    return delta_lat + delta_long + delta_alt


if __name__ == '__main__':

    # for arming and changing mode
    armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

    # for fetching current IMU data and GPS data
    imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, imu_callback)
    gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)

    # for setting target position in local and global frame
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    global_gps_pub = rospy.Publisher('mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)

    rospy.init_node("offboard_node")
    rate = rospy.Rate(20)

    pose = set_pose(z=2)
    gps_msg = set_gps(47.397839, 8.5457364, 540)

    print("Initializing ...")
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    idx = 100
    print ("taking off to: ", 2, "m")
    while not rospy.is_shutdown():

        if idx > 50:

            armService(True)
            isModeChanged = flightModeService(custom_mode='OFFBOARD')
            local_pos_pub.publish(pose)

        elif idx <= 50:

            armService(True)
            isModeChanged = flightModeService(custom_mode='OFFBOARD')
            global_gps_pub.publish(gps_msg)

            if distance(global_gps, gps_msg) < 0.05:
                print("Reached position!")


        # sleeping time should be less than 0.5s
        idx = idx -1
        time.sleep(0.1)
