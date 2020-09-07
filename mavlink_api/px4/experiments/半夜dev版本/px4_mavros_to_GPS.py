from mavros_msgs.msg import GlobalPositionTarget
import rospy
from mavros_msgs.srv import CommandBool,SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
import time
import math

#global variables
global_imu = Imu()
global_gps = NavSatFix()
current_state = State()
remote_control = False
ever_entered_offboard = False

def get_ever_entered_offboard():
    global ever_entered_offboard
    return ever_entered_offboard

def set_ever_entered_offboard(val):
    global ever_entered_offboard
    ever_entered_offboard = val

def set_remote_control(val):
    global remote_control
    remote_control = val

def get_remote_control():
    global remote_control
    return remote_control


def imu_callback(data):
    global global_imu
    global_imu = data


def gps_callback(data):
    global global_gps
    global_gps = data


def state_callback(data):
    
    ever_offboard = get_ever_entered_offboard()
    if ever_offboard:
        print ('OFFBOARD mode ever entered.Waiting for RC.')
        if data.mode != "OFFBOARD":
            print("Received mode: ",data.mode)
            current_state = data
            set_remote_control(True)
    else:
        if data.mode == "OFFBOARD":
            set_ever_entered_offboard(True)


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


# only linear speed
def set_speed(x=0.2,y=0.2,z=0.2):
    speed = Twist()
    speed.linear.x = x
    speed.linear.y = y
    speed.linear.z = z

    return speed
    


def distance(gps1, gps2):
    delta_lat = math.fabs(gps1.latitude-gps2.latitude)
    delta_long = math.fabs(gps1.longitude - gps2.longitude)
    delta_alt = math.fabs(gps1.altitude - gps2.altitude)

    return delta_lat + delta_long + delta_alt


if __name__ == '__main__':

    # for arming and changing mode
    armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    # for fetching current IMU data, GPS data and current mode
    imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, imu_callback)
    gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)
    state_sub = rospy.Subscriber("mavros/state", State, state_callback)

    # for setting target position in local and global frame, speed
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    global_gps_pub = rospy.Publisher('mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)
    speed_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

    rospy.init_node("offboard_node")
    rate = rospy.Rate(20)

    pose = set_pose(z=2)
    gps_msg = set_gps(43.81064, 125.4389666, 240)
    #gps_msg = set_gps(47.397822, 8.5456647, 540)

    print("Initializing ...")
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    speed = set_speed()
    speed_pub.publish(speed)
    print("Setting speed to: ", speed.linear.x, speed.linear.y, speed.linear.z)

    idx = 200
    print ("taking off to: ", 2, "m")

    #global remote_control
    armService(True)
    Enter_offboard_result = flightModeService(custom_mode = 'OFFBOARD')
    while not Enter_offboard_result:
        print 'enter offboard failed.retry.'
        Enter_offboard_result = flightModeService(custom_mode = 'OFFBOARD')
    while not get_ever_entered_offboard():
        print('Not in offboard mode. Waiting.')
        rate.sleep()
        
    while not get_remote_control():

        if idx > 150:
            #armService(True)
            #isModeChanged = flightModeService(custom_mode='OFFBOARD')
            local_pos_pub.publish(pose)

        elif idx <= 150:
            if idx == 150:
                print("Flying to GPS!")
            #armService(True)
            #isModeChanged = flightModeService(custom_mode='OFFBOARD')
            global_gps_pub.publish(gps_msg)

            if distance(global_gps, gps_msg) < 0.05:
                print("Reached position!")
        
        else:
            if idx==50:
                print("LAND!")
            #armService(True)
            isModeChanged = flightModeService(custom_mode='LAND')
            
        
        # sleeping time should be less than 0.5s
        idx = idx -1

        time.sleep(0.1)
    print('loop quit!')
