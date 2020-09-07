import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool,SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
import time
import math

#global variables
global_imu = Imu()
global_gps = NavSatFix()
local_pose = PoseStamped()
target_pose = PoseStamped()
target_raw = PositionTarget()
current_state = State()
remote_control = False
ever_entered_offboard = False

# for keeping the number of successes of finishing one job
num_of_success = 0

received_new_task = False

# get and set ever entered offboard variable
def get_ever_entered_offboard():
    global ever_entered_offboard
    return ever_entered_offboard

def set_ever_entered_offboard(val):
    global ever_entered_offboard
    ever_entered_offboard = val

# set and get remote control
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

def local_pose_callback(data):
    global local_pose
    local_pose = data


#------------------------------------------------------------------------------------------------------------------------------------------------
def local_setposition_callback(data):
    global target_pose, received_new_task, global_imu, target_raw

    target_raw = set_target_pose(x=data.pose.position.x, y=data.pose.position.y, z=data.pose.position.z)
    
    print "Received New Position Task!"
    received_new_task = True


def local_setorientation_callback(data):
    global target_pose, received_new_task, local_pose

    print "Received New Orientation Task!: ", data.data
   
    temp_q = Quaternion(axis=[0.0, 0.0, 1.0], degrees= float(data.data))
    cur_q = Quaternion(local_pose.pose.orientation.w, local_pose.pose.orientation.x, local_pose.pose.orientation.y, local_pose.pose.orientation.z)
    result = cur_q * temp_q

    target_pose.pose.orientation.w = result[0]
    target_pose.pose.orientation.x = result[1]
    target_pose.pose.orientation.y = result[2]
    target_pose.pose.orientation.z = result[3]
    
    target_pose.pose.position = local_pose.pose.position

    received_new_task = True

#--------------------------------------------------------------------------------------------------------------------------------------------------


def state_callback(data):
    
    ever_offboard = get_ever_entered_offboard()
    if ever_offboard:
        if data.mode != "OFFBOARD":
            print("Received mode: ",data.mode)
            current_state = data
            set_remote_control(True)
    else:
        if data.mode == "OFFBOARD":
            set_ever_entered_offboard(True)



def set_target_pose(x=0, y=0, z=2, use_current_heading=True):
    
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()

    #FRAME_LOCAL_NED
    #FRAME_LOCAL_OFFSET_NED
    #FRAME_BODY_OFFSET_NED
    target_raw_pose.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
    
    target_raw_pose.position.x = x
    target_raw_pose.position.y = y
    target_raw_pose.position.z = z
    
    target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.FORCE
    print "target_raw_pose: ",target_raw_pose

    target_raw_pose.yaw = 10
    target_raw_pose.yaw_rate = 1

    print "target_raw_pose: ", target_raw_pose

    return target_raw_pose


def set_gps(lat, long, alt):
    gps = GlobalPositionTarget()
    gps.header.stamp = rospy.Time.now()
    gps.latitude = float(lat)
    gps.longitude = float(long)
    gps.altitude = float(alt)
    return gps


# only linear speed
def set_speed(x=0.2,y=0.2,z=0.2):
    speed = Twist()
    speed.linear.x = x
    speed.linear.y = y
    speed.linear.z = z
    return speed
    

def gps_distance(gps1, gps2):
    delta_lat = math.fabs(gps1.latitude-gps2.latitude)
    delta_long = math.fabs(gps1.longitude - gps2.longitude)
    delta_alt = math.fabs(gps1.altitude - gps2.altitude)
    # gives lat and long more weight
    return 0.4*delta_lat + 0.4*delta_long + 0.2*delta_alt


def pose_distance(p1, p2):
    delta_x = math.fabs(p1.pose.position.x - p2.pose.position.x)
    delta_y = math.fabs(p1.pose.position.y - p2.pose.position.y)
    delta_z = math.fabs(p1.pose.position.z - p2.pose.position.z)
    return math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)



def do_task():
    global num_of_success, local_pose, target_pose

    local_pos_pub.publish(target_pose)

    #target_raw_pose = PositionTarget()
    #target_raw_pose.header = target_pose.header
    #target_raw_pose.position = target_pose.pose.position
    #target_raw_pose.coordinate_frame = target_raw_pose.FRAME_BODY_OFFSET_NED
    #local_target_pub.publish(target_raw_pose)
    
    #print "target_raw_pose: ", target_raw_pose

    if pose_distance(target_pose, local_pose) < 0.1:
        print "Reached preset target position!"
        return True

     

if __name__ == '__main__':

    # for arming and changing mode
    armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    # for fetching current IMU data, GPS data and current mode
    imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, imu_callback)
    state_sub = rospy.Subscriber("mavros/state", State, state_callback)
    gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)
    local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, local_pose_callback)

    # for setting new target posiiton and orientation
    local_setposition_sub = rospy.Subscriber("gi/set_pose/position", PoseStamped, local_setposition_callback)
    #local_setpos_pub = rospy.Publisher('gi/set_pose/pose', PoseStamped, queue_size=10)
    local_setorientation_sub = rospy.Subscriber("gi/set_pose/orientation", Float32, local_setorientation_callback)

    # for setting target position in local and global frame, speed
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    
    # will freefall immediately
    local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    global_gps_pub = rospy.Publisher('mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)
    speed_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)


    rospy.init_node("offboard_node")
    rate = rospy.Rate(20)
    
    a_target_pose = set_target_pose()

    print("Initializing ...")
    for i in range(1):
        #local_pos_pub.publish(pose)
        local_target_pub.publish(a_target_pose)
        rate.sleep()
    print("Initializing finished!")

    #arm vehicle
    armService(True)

    #try to enter offboard mode
    Enter_offboard_result = flightModeService(custom_mode = 'OFFBOARD')
    while not Enter_offboard_result:
        print 'enter offboard failed, retrying!.'
        Enter_offboard_result = flightModeService(custom_mode = 'OFFBOARD')

    while not get_ever_entered_offboard():
        print('Not in offboard mode. Waiting.')
        rate.sleep()

    #automatically takeoff no matter what
    while not received_new_task:
        #local_pos_pub.publish(pose)
        local_target_pub.publish(a_target_pose)

    #main loop, receive new pose and execute task!
    while (not get_remote_control()):
	
	local_target_pub.publish(target_raw)

        # sleeping time should be less than 0.2s
        time.sleep(0.02)


    print('Exited main loop!')
