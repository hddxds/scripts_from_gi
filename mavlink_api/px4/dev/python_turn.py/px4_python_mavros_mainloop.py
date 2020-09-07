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


def local_pose_callback(data):
    global local_pose
    local_pose = data


def local_setorientation_callback(data):
    global target_pose, received_new_task, local_pose

    print "Received New Orientation Task!: ", data.data
   
    temp_q = Quaternion(axis=[0.0, 0.0, 1.0], degrees= float(data.data))
    cur_q  = Quaternion(local_pose.pose.orientation.w, local_pose.pose.orientation.x, local_pose.pose.orientation.y, local_pose.pose.orientation.z)
    result = cur_q * temp_q

    target_pose.pose.orientation.w = result[0]
    target_pose.pose.orientation.x = result[1]
    target_pose.pose.orientation.y = result[2]
    target_pose.pose.orientation.z = result[3]
    
    target_pose.pose.position = local_pose.pose.position

    received_new_task = True

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


def set_pose(x=0, y=0, z=2, use_current_heading=True):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    if (use_current_heading):
        pose.pose.orientation = global_imu.orientation
    else:
        print("Not using current heading!")

    return pose


def pose_distance(p1, p2):
    delta_x = math.fabs(p1.pose.position.x - p2.pose.position.x)
    delta_y = math.fabs(p1.pose.position.y - p2.pose.position.y)
    delta_z = math.fabs(p1.pose.position.z - p2.pose.position.z)
    return math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)



def do_task():
    global num_of_success, local_pose, target_pose

    local_setorientation_callback.publish(target_pose)

    if pose_distance(target_pose, local_pose) < 0.1:
        print "Reached preset target position!"
        return True

        

if __name__ == '__main__':

    rospy.init_node("offboard_turn")
    rate = rospy.Rate(20)

    # for fetching current IMU data, GPS data and current mode
    imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, imu_callback)
    state_sub = rospy.Subscriber("mavros/state", State, state_callback)

    local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, local_pose_callback)

    # for setting new target posiiton and orientation
    local_setposition_sub = rospy.Subscriber("gi/set_pose/position", PoseStamped, local_setposition_callback)
    #local_setpos_pub = rospy.Publisher('gi/set_pose/pose', PoseStamped, queue_size=10)
    local_setorientation_sub = rospy.Subscriber("gi/set_pose/orientation", Float32, local_setorientation_callback)

    # for setting target position in local and global frame, speed
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    
    # will freefall immediately
    #local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    global_gps_pub = rospy.Publisher('mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)

    #main loop, receive new pose and execute task!
    while True:


        do_task()

        # sleeping time should be less than 0.2s
        time.sleep(0.1)


    print('Exited main loop!')
