#encoding=utf-8
from astar import A_star
from multiprocessing import Queue,Process
from time import sleep
import rospy


class Driver:
    def __init__(self): #init_pos, end_pos,command_queue,returnval_queue):
        self.TIME_DELAY_THRESHOLD = 500 # TODO: fix this.

        #to keep a obstacle set buffer.
        self.obstacle_set = set()
        self.obs_set_last_update_time = rospy.rostime.get_time()

    def set_obstacle_set(self, obstacle_set):
        self.obstacle_set = obstacle_set
        self.obs_set_last_update_time = rospy.rostime.get_time()

    def get_obstacles_around(self):
        current_time = rospy.rostime.get_time()
        if current_time - self.obs_set_last_update_time > self.TIME_DELAY_THRESHOLD:
            print("Warning:Buffer timeout!Delay:", current_time - self.obs_set_last_update_time)
        return self.obstacle_set


