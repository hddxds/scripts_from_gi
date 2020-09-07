from mavros_msgs.msg import GlobalPositionTarget
import rospy
from mavros_msgs.srv import *
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import time


global_imu = Imu()

def imu_callback(data):
    global global_imu
    global_imu = data


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


if __name__ == '__main__':

    # for arming and changing mode
    armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

    # for fetching current IMU data
    imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, imu_callback)

    # for setting target position in local and global frame
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    rospy.init_node("offboard_node")
    rate = rospy.Rate(20)

    pose = set_pose(z=2)

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
            if(idx==50):
                print( "landing!")
            armService(True)
            isModeChanged = flightModeService(custom_mode='AUTO.LAND')

        # sleeping time should be less than 0.5s
        idx = idx -1
        time.sleep(0.1)
