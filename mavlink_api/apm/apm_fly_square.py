#!/usr/bin/env python

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil 
import time
import math

#do not execute the script while the QGC is opened because there will be warnings.


class controlAPM:

    def __init__(self, con_port="/dev/ttyACM0"):
        self.connection_string = con_port
        self.vehicle = connect(self.connection_string, wait_ready=True)
        time.sleep(10)


    def task(self, position_x, position_y, position_z, heading, sleep_time=5, is_relative=True):
        #send guided anyway, won't hurt
        self.vehicle.mode = VehicleMode("GUIDED")
        #1st
        msg1 = self.vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111111000,
        position_x,  #forward +
        position_y,   #right   +
        position_z,   #down    +
        0.5, 0.5, 0.5, 0,0,0, 0,0)
        self.vehicle.send_mavlink(msg1)
        #2nd, make sure it reach preset position
        time.sleep(sleep_time)
        #--------------------------------------------------------------------
        #3rd turn
        msg2 = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        self.vehicle.send_mavlink(msg2)
        #4th, make sure it reach preset position
        time.sleep(sleep_time)
        print ("task finished!")


    def disconnect(self):
        self.vehicle.close()



if __name__ == '__main__':

    con = controlAPM()

    con.task(1, 0, 0, 90)
    con.task(1, 0, 0, 90)
    con.task(1, 0, 0, 90)
    con.task(1, 0, 0, 90)

    con.disconnect()
	
