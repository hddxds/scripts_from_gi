//
// Created by gishr on 19-6-12.
//

#ifndef CONTROL_H
#define CONTROL_H

//#include

#include "rrt_star.h"

#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <math.h>
#include <thread>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/duration.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


using namespace Eigen;
using namespace std;

class control{

public:

    control(float grid_size=0.1);

    void start();

    void set_obstacle(sensor_msgs::PointCloud2& ros_pointcloud2);


private:
    ros::NodeHandle mNH;
    ros::Subscriber mPointcloud_sub;
    ros::Subscriber mPose_sub;
    ros::Subscriber mState_sub;

    ros::Publisher mMove_pub;

public:

    void ros_thread();

    void pointcloud_sub(const sensor_msgs::PointCloud2& pointcloud);

    void pose_sub(const geometry_msgs::PoseStamped &posemsg);

    void state_sub(const mavros_msgs::State& state);

public:
    float target_distance(Vector3f& target_pose);

    void move(Vector3f way_point, bool local_frame=true);

    bool fly_by_path(vector<Vector3f>& path);

    Vector3f discrete2continuous(Vector3f& discrete);

    Vector3f continuous2discrete(Vector3f& continuous);

    geometry_msgs::PoseStamped construct_pose(Vector3f& pose, bool body_frame);

private:

    vector<Vector3f> mObstacleDiscrete;

    vector<Vector3f> mPath;

    Vector3f mPoseD, mTargetD, mPoseC, mTargetC;

    float mGridSize;

    geometry_msgs::PoseStamped mCurPoseROS;

    sensor_msgs::PointCloud2 mCurPointCloudROS;

    mavros_msgs::State mStateROS;

    // path planning packages
    shared_ptr<rrt_star> mRRTstar = nullptr;

    shared_ptr<thread> mRosThred;

    std::thread* mThread;

};

#endif //CONTROL_H
