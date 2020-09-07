import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool,SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String
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
global_target_yaw = 0

initial_heading = math.pi / 2.0
current_heading = -1

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
    global global_imu, current_heading
    global_imu = data

    current_heading = q2euler(global_imu.orientation)

def gps_callback(data):
    global global_gps
    global_gps = data

def local_pose_callback(data):
    global local_pose
    local_pose = data


def state_callback(data):
    ever_offboard = get_ever_entered_offboard()
    if ever_offboard:
        if data.mode != "OFFBOARD":
            print("Received mode: ", data.mode)
            current_state = data
            set_remote_control(True)
    else:
        if data.mode == "OFFBOARD":
            set_ever_entered_offboard(True)

#------------------------------------------------------------------------------------------------------------------------------------------------

def q2euler(q):
    if isinstance(q,Quaternion):
        rotate_z_rad = q.yaw_pitch_roll[0]
    else:
        q_ = Quaternion(q.w,q.x,q.y,q.z)
        rotate_z_rad = q_.yaw_pitch_roll[0]

    '''
    cos_halftheta = w
    rotate_z_rad=0
    if cos_halftheta > 0:
        sin_halftheta = math.sqrt(1-cos_halftheta**2)
        cos_beta_z = z/sin_halftheta
        rotate_z_rad = math.acos(cos_beta_z)
    else:#w<0 let x,y,z of q be the minus version.
        w,x,y,z = -w,-x,-y,-z
        cos_halftheta = w
        sin_halftheta = math.sqrt(1-cos_halftheta**2)
        cos_beta_z = z/sin_halftheta
        rotate_z_rad = math.acos(cos_beta_z)'''
    rotate_z_deg = (rotate_z_rad*180)/math.pi
    return rotate_z_deg


def body2ned(body_target_x, body_target_y, body_target_z):
    global initial_heading, global_target_yaw, local_pose

    heading_delta = initial_heading - global_target_yaw

    NED_y = body_target_y * math.cos(heading_delta) - body_target_x * math.sin(heading_delta) + local_pose.pose.position.y
    NED_x = body_target_y * math.sin(heading_delta) + body_target_x * math.cos(heading_delta) + local_pose.pose.position.x
    NED_z = body_target_z + local_pose.pose.position.z

    print ("original xyz are:", body_target_x, body_target_y, body_target_z)
    print ("new xyz are:", NED_x, NED_y, NED_z)
    print ("heading delta is:", heading_delta)

    return NED_x, NED_y, NED_z

'''
def local_heading_callback(data):
    global current_heading

    current_heading = data.data
'''

def local_setposition_callback(data):
    global target_pose, received_new_task, global_imu, target_raw, local_pose

    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z

    new_x, new_y, new_z = body2ned(x, y, z)
    #new_x, new_y, new_z = (data.pose.position.x, data.pose.position.y, data.pose.position.z)

    target_raw = set_target_pose(x=new_x, y=new_y, z=new_z)
    
    #print "target_raw:", target_raw
    print "Received New Position Task!"
    received_new_task = True


def local_setorientation_callback(data):
    global target_pose, received_new_task, local_pose, target_raw, global_target_yaw

    print "Received New Orientation Task!: ", data.data
   
    temp_q = Quaternion(axis=[0.0, 0.0, 1.0], degrees= float(data.data))
    #cur_q = Quaternion(local_pose.pose.orientation.w, local_pose.pose.orientation.x, local_pose.pose.orientation.y, local_pose.pose.orientation.z)
    #result = cur_q * temp_q
    result = temp_q # abs rotate.

    yaw = q2euler(result)
    global_target_yaw = yaw * math.pi / 180.0
    t = PositionTarget()

    # t.pose.orientation.w = result[0]
    # t.pose.orientation.x = result[1]
    # t.pose.orientation.y = result[2]
    # t.pose.orientation.z = result[3]

    t.yaw = global_target_yaw

    t.position = local_pose.pose.position
    t.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.FORCE

    target_raw = t
    #local_target_pub.publish(t)

    received_new_task = True


#--------------------------------------------------------------------------------------------------------------------------------------------------
def set_target_pose(x=0, y=0, z=2, use_current_heading=True):

    #global current_heading
    global global_target_yaw

    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()

    #FRAME_LOCAL_NED
    #FRAME_LOCAL_OFFSET_NED
    #FRAME_BODY_OFFSET_NED
    target_raw_pose.coordinate_frame = 9
    
    target_raw_pose.position.x = x
    target_raw_pose.position.y = y
    target_raw_pose.position.z = z
    
    target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.FORCE
    #print "target_raw_pose: ",target_raw_pose

    target_raw_pose.yaw = global_target_yaw #current_heading
    target_raw_pose.yaw_rate = 1

    return target_raw_pose


def pose_distance(p1, p2):
    delta_x = math.fabs(p1.pose.position.x - p2.pose.position.x)
    delta_y = math.fabs(p1.pose.position.y - p2.pose.position.y)
    delta_z = math.fabs(p1.pose.position.z - p2.pose.position.z)
    return math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)


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

    # heading callback
    #heading_sub = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, local_heading_callback)


    # for setting target position in local and global frame, speed
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    
    local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    global_gps_pub = rospy.Publisher('mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)
    speed_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)


    rospy.init_node("offboard_node")
    rate = rospy.Rate(20)
    
    a_target_pose = set_target_pose()

    print("Initializing ...")
    for i in range(100):
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
    global target_raw



    #main loop, receive new pose and execute task!
    print('OFFBOARD MODE ENTERED!')
    while (not get_remote_control()):
        local_target_pub.publish(target_raw)

        # sleeping time should be less than 0.2s
        time.sleep(0.02)


    print('Exited main loop!')
