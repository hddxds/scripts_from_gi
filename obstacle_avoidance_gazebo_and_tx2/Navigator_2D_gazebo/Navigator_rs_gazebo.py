#encoding=utf-8

'''
        project overview:

	Subscribe:
		1.slam pose(global/local pose) *
		2.octomap_server/global map
		3.local pointcloud/local octomap
		4.target input(semantic target/visual pose target/gps target)
	Publish:
		1.Mavros(amo) Command
		2.Navigator status

	Algorithms:
		1.D*
		2.state transfer
		3.position->position PID controller
		4.global/semantic/visual target to local pose
'''

import threading
import time
from path_optimization.path_pruning import PathPruning
# for ros
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Imu, NavSatFix, PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker,MarkerArray
# for mavros
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget#, Command
from mavros_msgs.srv import CommandBool, SetMode

# for octomap
from octomap_msgs.msg import Octomap, OctomapWithPose, octomap_msgs
from helper import save_points3D, load_points3D

# other useful utilities
#from pyquaternion import Quaternion
import pyquaternion

import astar.astar
import astar.driver

import time
import math
from enum import Enum
import thread
#from queue import Queue

#from Pos2PosController import Pos2PosController as Controller  # TODO:re-implement this.
from SimController import Controller as Controller
import DiscreteGridUtils
import numpy as np
from RandomSampling.randomsampling import randomsampling



# define system status
class status(Enum):

    INITIALIZED = 1
    LOOKING_FOR_PATH = 2
    LOOKING_FOR_PATH_SUCCEED = 3
    LOOKING_FOR_PATH_FAILED = 4
    GOING_TO_TARGET = 5
    GOING_TO_VISION_TARGET = 6



class Navigator:
    
    def __init__(self, save_pts=False, config_file_path=None):
        if config_file_path:
            pass

        rospy.init_node("gi_navigator_node")
        self.dg = DiscreteGridUtils.DiscreteGridUtils(grid_size=0.1)  # modify grid size according to different purposes
        self.rate = rospy.Rate(50)
        self.driver = astar.driver.Driver()
        self.controller = Controller()
        self.mavros_state = "OFFBOARD"
        self.set_status(status.INITIALIZED)
        self.save_pts = save_pts

        self.cur_command_id = 0
        self.prev_command_id = 0
        self.cur_target_position = None

        self.task_id = -1
        self.obstacle_set_mutex = threading.Lock()  # mutex.acquire(timeout);mutex.release()
        self.nav_command_mutex = threading.Lock()  # for nav command in dstar and ros high level command.
        self.local_pose_d = None
        self.local_pose_c = None

        self.navigator_status_pub = rospy.Publisher('/gi/navigator_status', String, queue_size=10)
        self.path_plan_pub = rospy.Publisher('/gi/navi_path_plan', MarkerArray, queue_size=10)

        self.path = []
        self.path_prune = PathPruning(obstacle_distance=12)

        self.rs = randomsampling()

        t1 = threading.Thread(target=self.ros_thread)
        t1.start()

    '''
    Navigating thread    
    '''
    def keep_navigating(self):

        while self.mavros_state == "OFFBOARD" and not (rospy.is_shutdown()):

            # get target position and local position in discrete
            current_pos = self.get_local_pose_d()
            end_pos = self.get_latest_target()

            if current_pos is None:
                print('current pose not valid!')
                time.sleep(0.5)
                continue

            while not self.reachTargetPositionDiscrete(end_pos, 4) \
                    and (not self.navi_task_terminated()) \
                    and (not rospy.is_shutdown()):  # Till task is finished:

                print('From ', self.get_local_pose_d())

                # This is the place where you modify path planner
                t1 = time.time()
                self.set_status(status.LOOKING_FOR_PATH)
                print("start and end are: ", self.get_local_pose_d(), end_pos)

                temp_obs = self.driver.get_obstacles_around()
                if(temp_obs is not None):
                    _, self.path = self.rs.find_path(pose_d=self.get_local_pose_d(), target_pose_d=end_pos, obstacle_3d=temp_obs)
                    t2 = time.time()
                    print('random sampling path finding time cost:', (t2 - t1))

                if not self.path:
                    self.set_status(status.LOOKING_FOR_PATH_SUCCEED)
                    print('No path found!, self.path is None')
                    time.sleep(0.05)
                else:
                    # Path found. keep state machine and do task step by step.
                    print("Path found!")
                    self.publish_path(self.path, (1, 0, 0))
                    self.set_status(status.LOOKING_FOR_PATH_FAILED)

                    # going through each waypoint
                    for next_move in self.path:
                        if self.navi_task_terminated():
                            break

                        print('next_move : ', next_move)

                        # if next way point is in collision with obstacles
                        # if not self.driver.algo.is_valid(next_move, self.driver.get_obstacles_around()):
                        #     print('Next waypoint is in collision with obstacle, path not valid!')
                        #     break

                        next_position_continuous = self.dg.discrete_to_continuous_target(next_move)

                        print("local pose: ", self.local_pose_c)
                        print("target pose: ", next_position_continuous)

                        while not self.reachTargetPositionContinuous(next_position_continuous, 0.5):
                            self.controller.mav_move(next_position_continuous[0],
                                                     next_position_continuous[1],
                                                     next_position_continuous[2],
                                                     abs_mode=True)
                            time.sleep(0.05)


            print("Target Reached!")
            time.sleep(0.05)  # wait for new nav task.

        print("Mavros not in OFFBOARD mode, Disconnected!")


    '''
    move quad in body frame
    '''

    def terminate_navigating(self):
        #TODO
        pass

    def resume_navigating(self):
        #TODO
        pass

    def set_target_position(self, target_position):
        if target_position and len(target_position) is 3:
            self.cur_target_position = self.dg.continuous_to_discrete(target_position)

    def get_latest_target(self):
        return self.cur_target_position

    def set_vision_target(self, vision_target):
        self.set_status(status.GOING_TO_VISION_TARGET)
        self.set_target_position(vision_target)

    def navi_task_terminated(self):
        if self.dist(self.local_pose_d, self.cur_target_position) < 2:  #TODO: or stop flag is set.
            return True
        else:
            return False

    '''
    Dstar Thread
    
    def Dstar_thread(self):
        while not rospy.is_shutdown():
            while status!= xxx:# TODO
                next_move = xxx
                return next_move'''

    '''##For test:
        target = [0.5, 0.5, 0.5]
        self.set_target_postion(target)
        pass'''

    '''
    ROS thread
    responsible for subscribers and publishers
    '''
    def ros_thread(self):
        print('ros_thread spawn!!!!')
        self.octomap_msg = None

        # subscribers
        self.slam_sub = rospy.Subscriber("/gi/slam_output/pose", PoseStamped, self.slam_pose_callback)
        self.vision_target_sub = rospy.Subscriber("/gi/visual_target/pose", PoseStamped, self.vision_target_callback)
        self.point_cloud_sub = rospy.Subscriber("/camera/left/point_cloud", PointCloud, self.point_cloud_callback)
        self.octomap_cells_vis = rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, self.octomap_update_callback)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)


        # publishers
        #self.mavros_control_pub = rospy.Publisher('mavros/Command', Command, queue_size=10)

        self.set_status(status.INITIALIZED)

        rospy.spin()


    '''
    ROS callbacks
    '''
    def slam_pose_callback(self, msg):
        self.slam_pose = msg

    def vision_target_callback(self, msg):
        self.vision_target = msg
        #print("Received New Vision Target!")

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode
        self.navigator_status_pub.publish(self.STATUS)

    def point_cloud_callback(self, msg):
        self.current_point_cloud = msg

    def octomap_update_callback(self, msg):  # as pointcloud2.
        obs_set = set()
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            #print " x : %f  y: %f  z: %f" % (p[0], p[1], p[2])
            point = self.dg.continuous_to_discrete((p[0], p[1], p[2]))
            #print("corresponding discrete value: ", point)
            obs_set.add(point)

        # save points set
        if(self.save_pts):
            save_points3D(obs_set)

        acquired = self.obstacle_set_mutex.acquire(True)  # blocking.
        if acquired:
            #print('octomap updated!')
            self.driver.set_obstacle_set(obs_set)
            self.obstacle_set_mutex.release()
            return
        else:
            print('Lock not acquired!')

    def local_pose_callback(self, msg):
        pose_ = msg.pose.position  #TODO:do fusion with visual slam.

        self.local_pose_c = (pose_.x, pose_.y, pose_.z)
        self.local_pose_d = self.dg.continuous_to_discrete((pose_.x, pose_.y, pose_.z))

    # return pose in discrete
    def get_local_pose_d(self):  # in mavros axis.for command.
        return self.local_pose_d

    # return pose in continuous
    def get_local_pose_c(self):
        return self.local_pose_c

    '''
    helper functions
    '''
    def set_status(self, status):
        self.STATUS = String(status.name)


    def dist(sefl, pos1, pos2):
        if not pos1 or not pos2:
            return False, 0
        else:
            return True, reduce(lambda x, y: x + y, map(lambda i: (pos1[i] - pos2[i]) ** 2, [0, 1, 2]))


    # target should be Continuous
    def reachTargetPositionContinuous(self, target, threshold=0.7):

        delta_x = math.fabs(self.local_pose_c[0] - target[0])
        delta_y = math.fabs(self.local_pose_c[1] - target[1])
        delta_z = math.fabs(self.local_pose_c[2] - target[2])

        distance = (delta_x + delta_y + delta_z)

        print("distance: ", distance, "threshold: ", threshold)
        if distance < threshold:
            return True
        else:
            return False

    # target should be discrete
    def reachTargetPositionDiscrete(self, target, threshold=3):

        delta_x = math.fabs(self.local_pose_d[0] - target[0])
        delta_y = math.fabs(self.local_pose_d[1] - target[1])
        delta_z = math.fabs(self.local_pose_d[2] - target[2])

        distance = (delta_x + delta_y + delta_z)

        print("distance: ", distance, "threshold: ", threshold)
        if distance < threshold:
            return True
        else:
            return False


    def setMavMode(self, msg):
        pass

    def do_hover(self):
        pass


    def publish_path(self, path, RGB=(1, 0, 0)):
        m_arr = MarkerArray()
        marr_index = 0
        for next_move in path:
            point = self.dg.discrete_to_continuous_target((next_move[0], next_move[1], next_move[2]))
            mk = Marker()
            mk.header.frame_id = "map"
            mk.action = mk.ADD
            mk.id = marr_index
            marr_index += 1
            mk.color.r = RGB[0]
            mk.color.g = RGB[1]
            mk.color.b = RGB[2]
            mk.color.a = 1.0
            mk.type = mk.CUBE
            mk.scale.x = 0.3
            mk.scale.y = 0.3
            mk.scale.z = 0.3
            mk.pose.position.x = point[0]
            mk.pose.position.y = point[1]
            mk.pose.position.z = point[2]
            m_arr.markers.append(mk)
        self.path_plan_pub.publish(m_arr)



if __name__ == '__main__':
    nav = Navigator(False)

    # target position should be of size meter
    nav.set_target_position((4, 0, 1))
    nav.keep_navigating()
