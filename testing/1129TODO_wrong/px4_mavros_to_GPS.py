import rospy
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool,SetMode, CommandHome
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
import time
import math
import pyquaternion as q

#global variables
global_imu = Imu()
global_gps = NavSatFix()
local_pose = PoseStamped()
current_state = State()
remote_control = False
ever_entered_offboard = False

# for keeping the number of successes of finishing one job
num_of_success = 0

# initial_gps
num_gps_received = 0
home_gps = NavSatFix()
home_lat = 0.0
home_long = 0.0
home_alt = 0.0


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
    global global_gps, home_lat, home_long, home_alt, num_gps_received
    global_gps = data

    if num_gps_received < 10:
        home_lat = home_lat + data.latitude
        home_long = home_long + data.longitude
        home_alt = home_alt + data.altitude 
        num_gps_received = num_gps_received + 1
    
    elif num_gps_received == 10:
        home_lat = home_lat / 10.0
        home_long = home_long / 10.0
        home_alt = home_alt / 10.0
        num_gps_received = num_gps_received + 1
        print "Home gps set to:", home_lat, home_long, home_alt
        
        set_home(using_current_position = True)


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


def local_pose_callback(data):
    global local_pose
    local_pose = data


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


def set_gps(lat, long, alt):
    gps = GlobalPositionTarget()
    gps.header.stamp = rospy.Time.now()
    gps.latitude = float(lat)
    gps.longitude = float(long)
    gps.altitude = float(alt)
    return gps

def set_yaw(yaw):
    global global_imu

    gps = GlobalPositionTarget()
    gps.header.stamp = rospy.Time.now()

    imu_q = q.Quaternion(global_imu.orientation.w, global_imu.orientation.x, global_imu.orientation.y, global_imu.orientation.z)
    yaw_q = q.Quaternion(axis=[0,0,1], angle=yaw)
    new_q = yaw_q * imu_q

    print "imu_q", imu_q
    print "yaw_q", yaw_q
    print "new_q", yaw_q

    gps.latitude = global_gps.latitude
    gps.longitude = global_gps.longitude
    gps.altitude = global_gps.altitude

    gps.yaw = new_q[-1]

    return gps


# only linear speed
def set_speed(x=0.2,y=0.2,z=0.2):
    speed = Twist()
    speed.linear.x = x
    speed.linear.y = y
    speed.linear.z = z
    return speed
    

def set_home(using_current_position, lat=-1, long=-1, alt=-1):
    global global_gps, home_lat, home_long, home_alt
    if using_current_position:
        result = setHomeService(True, home_lat, home_long, home_alt)
        if result:
            print "Set home succeed!"
        else:
            print "Set home failed!"
    else:
        assert(lat > 0 and long > 0 and alt > 0)
        result = setHomeService(True, lat, long, alt)
        if result:
            print "Set home succeed!"
        else:
            print "Set home failed!"
        

# 1e5 -> accuracy in 1 meter
# 1e6 -> accuracy in 0.1 meter
def gps_distance(gps1, gps2):
    delta_lat = math.fabs(gps1.latitude-gps2.latitude)
    delta_long = math.fabs(gps1.longitude - gps2.longitude)
    delta_alt = math.fabs(gps1.altitude - gps2.altitude)
    # gives lat and long more weight
    return 0.5*1e5*delta_lat + 0.5*1e5*delta_long + delta_alt


def pose_distance(p1, p2):
    delta_x = math.fabs(p1.pose.position.x - p2.pose.position.x)
    delta_y = math.fabs(p1.pose.position.y - p2.pose.position.y)
    delta_z = math.fabs(p1.pose.position.z - p2.pose.position.z)
    return math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)


def readCSV(path):
    file = open(path).readlines()
    tasks = []
    for line in file:
        tasks.append(line.split('\n')[0].split(','))
    print "There are ", len(tasks), "tasks in total!"
    for idx, task in enumerate(tasks):
        print "Task", idx, ":", task

    return tasks
    

def do_task(task_list, id):  # id is the line num of csv file.

    global num_of_success, global_gps, home_lat, home_long, home_alt

    if task_list[id][0] == "takeoff":

        pose = set_pose(z=float(task_list[id][1]) )
        height_pub.publish(pose)
        #print "pose_distance:", pose_distance(local_pose, pose)
        
        if pose_distance(local_pose, pose) < 0.010:
            num_of_success = num_of_success + 1
            if num_of_success >= 50:
                print "Taking off succeed!"
                num_of_success = 0
                return True
            else:
                return False
        else:
            return False


    if task_list[id][0] == "gps":
        
        #assert(float(task_list[id][3])>0)

        gps = set_gps(task_list[id][1], task_list[id][2], home_alt + float(task_list[id][3]) )
        global_gps_pub.publish(gps)
        print "gps distance:", gps_distance(global_gps, gps)

        if gps_distance(global_gps, gps) < 0.01:
            num_of_success = num_of_success + 1
            if num_of_success >= 50:
                print "reached gps task", task_list[id]
                num_of_success = 0
                return True
            else:
                return False

        else:
            return False


    if task_list[id][0] == "yaw":

        gps_yaw = set_yaw(float(task_list[id][1]) )
        global_gps_pub.publish(gps_yaw)
        
        num_of_success = num_of_success + 1
        if num_of_success >= 5000:
            print "reached yaw", task_list[id]
            num_of_success = 0
            return True
        else:
            return False


    if task_list[id][0] == "xyz":

        pose = set_pose(x=float(task_list[id][1]), y=float(task_list[id][2]), z=float(task_list[id][3]) )
        local_pos_pub.publish(pose)
        print "pose_distance:", pose_distance(local_pose, pose)        


        if pose_distance(local_pose, pose) < 0.3:
            num_of_success = num_of_success + 1
            if num_of_success >= 50:
                print "task", task_list[id], "succeed!"
                num_of_success = 0
                return True
            else:
                return False
        else:
            return False
                    

    if task_list[id][0] == "land":
        isModeChanged = flightModeService(custom_mode='AUTO.LAND')
        return False

    # not recommanded for now as the drone will fly 30 meters above ground
    # which is dangerous
    if task_list[id][0] == "rtl":
        isModeChanged = flightModeService(custom_mode='AUTO.RTL')
        return False
        

if __name__ == '__main__':

    # for arming and changing mode
    armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    setHomeService = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)

    # for fetching current IMU data, GPS data and current mode
    imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, imu_callback)
    gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)
    state_sub = rospy.Subscriber("mavros/state", State, state_callback)
    local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, local_pose_callback)

    # for setting target position in local and global frame, speed
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    global_gps_pub = rospy.Publisher('mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)
    speed_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
    
    height_pub = rospy.Publisher("mavros/setpoint_attitude/attitude", PoseStamped, queue_size=10)   

    rospy.init_node("offboard_node")
    rate = rospy.Rate(20)

    pose = set_pose(z=2)

    print("Initializing ...")
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    print("Initializing finished!")

    #speed = set_speed()
    #speed_pub.publish(speed)
    #print("Setting speed to: ", speed.linear.x, speed.linear.y, speed.linear.z)

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

    tasks = readCSV('/home/gishr/software/px4/guidedAPM/scripts/testing/1129TODO_wrong/tasks')
    task_id = 0
    
    #--------------------------------------------------------------------------
    while not get_remote_control():

        if not do_task(tasks, task_id):
            print "number of success", num_of_success
            continue

        else:
            print "Go to the next task!"
            task_id = task_id + 1

        rate.sleep()
        time.sleep(0.1)
    #--------------------------------------------------------------------------    


    print('Exited main loop!')
