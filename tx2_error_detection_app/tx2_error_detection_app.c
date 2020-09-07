/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_local_position.h>

__EXPORT int tx2_error_detection_main(int argc, char *argv[]);


//TODO.do something like beep inside of HANDLE_ERROR.

#define HANDLE_ERROR(...)\
do{\
	PX4_ERR(__VA_ARGS__);\
}while(0)\

int tx2_error_detection_main(int argc, char *argv[])
{
	//PX4_INFO("Hello Sky!");
	PX4_INFO("TX2 error detection app started!");
	PX4_INFO("Caonima!");
	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
        //int tx2_heartbeat_sub_fd = orb_subscribe(ORB_ID(topic_tx2_heartbeat));
        //#define topic_tx2_slam_info vehicle_vision_position
        int tx2_slam_info_sub_fd = orb_subscribe(ORB_ID(vehicle_vision_position));

        //int gps_sub_fd = orb_subscribe(ORB_ID(vehicle_gps_position));
	/* limit the update rate to 5 Hz */
	//orb_set_interval(sensor_sub_fd, 200);
	orb_set_interval(sensor_sub_fd, 10);


	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = tx2_slam_info_sub_fd, .events = POLLIN},
		//{ .fd = tx2_heartbeat_sub_fd, .events = POLLIN} //TODO:Not implemented.Need a tx2 app.
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};
        const int sensors_sub_index = 0;
	const int tx2_slam_sub_index = 1;
        //const int tx2_heartbeat_sub_index = 2;

	int error_counter = 0;
        int time_ms_timeout = 100; //timeout:100ms.
	while(true) {
		/* wait for sensor update of 3 file descriptor */ //for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 3, time_ms_timeout);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			HANDLE_ERROR("ERROR NO MSG GOT:Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				HANDLE_ERROR("ERROR IN POLL FUNCTION:return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {
			if (!(fds[sensors_sub_index].revents & POLLIN))
			{
				HANDLE_ERROR("ERROR:SENSORS MSG NOT HEARD!");
			}
			//if (!(fds[tx2_heartbeat_sub_index].revents & POLLIN))
			//{
			//	HANDLE_ERROR("ERROR:TX2 HEARTBEAT DROPED!")
			//}
			if (!(fds[tx2_slam_sub_index].revents & POLLIN))
			{
				HANDLE_ERROR("ERROR:TX2 SLAM INFO DROPED!");
			}
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	HANDLE_ERROR("Error:error detection app itself caught into error, will exit!");

	return 0;
}
