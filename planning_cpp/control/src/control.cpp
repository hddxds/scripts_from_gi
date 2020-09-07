//
// Created by gishr on 19-6-12.
//

#include "control.h"


control::control(float grid_size){

    mGridSize = grid_size;

    mRRTstar = make_shared<rrt_star>(10);

    mMove_pub = mNH.advertise<geometry_msgs::PoseStamped>("gi/set_pose/position", 1, this);

//    thread ros_tread(&control::ros_thread, this);
//    mRosThred = make_shared<thread>(&control::ros_thread, this);
//    mRosThred->detach();

    mThread = new thread(&control::ros_thread, this);
};

void control::ros_thread(){

    mPointcloud_sub = mNH.subscribe("/cloud_in", 1, &control::pointcloud_sub, this);
    mPose_sub = mNH.subscribe("/mavros/local_position/pose", 1, &control::pose_sub, this);
    mState_sub = mNH.subscribe("/mavros/state", 1, &control::state_sub, this);

    ros::spin();
}

void control::start() {

    cout<<"Control Node Started!"<<endl;
    cout<<"mState.mode: "<<mStateROS.mode<<endl;
    cout<<"mStateROS.connected: "<<mStateROS.connected<<endl;

//    while (mStateROS.connected && mStateROS.mode == "OFFBOARD"){
    while (1) {

        auto path = mRRTstar->find_path(Vector3f(0, 0, 15), Vector3f(60, 0, 15), mObstacleDiscrete);
        if (path.empty()) {
            cout << "Path not found!" << endl;
            continue;
        } else {
            cout<<"Path found, mObstacleDiscrete size: "<<mObstacleDiscrete.size()<<endl;
            cout << "Path found!" << endl;
            mPath = path;
            mRRTstar->visualize_world();
        }

        bool result = fly_by_path(mPath);
        if (result) {
            cout << "We have reached target!" << endl;
        }
    }
//    mRosThred->join();
}

//void control::start() {
//
//    cout<<"Control Node Started!"<<endl;
//    cout<<"mState.mode: "<<mStateROS.mode<<endl;
//    cout<<"mStateROS.connected: "<<mStateROS.connected<<endl;
//    cout<<"mCurPointCloud.points.size(): "<<mCurPointCloudROS.points.size()<<endl;
//
//
////    while (mStateROS.connected && mStateROS.mode == "OFFBOARD"){
//    while (1){
//        if(!mObstacleDiscrete.empty()){
//
//            auto path = mRRTstar->find_path(Vector3f(0, 0, 0), Vector3f(40, 0, 5), mObstacleDiscrete);
//            if(path.empty()){
//                cout<<"Path not found!"<<endl;
//                continue;
//            }
//            else{
//                cout<<"Path found!"<<endl;
//                mPath = path;
//                mRRTstar->visualize_world();
//            }
//
//            bool result = fly_by_path(mPath);
//            if(result){
//                cout<<"We have reached target!"<<endl;
//            }
//        }
//        else{
//            cout<<"mObstacleDiscrete empty, waiting"<<endl;
//        }
//    }
//
////    mRosThred->join();
//}

geometry_msgs::PoseStamped control::construct_pose(Vector3f& pose, bool body_frame){
    geometry_msgs::PoseStamped ros_pose;
    ros_pose.header.stamp = ros::Time::now();
    if(body_frame)
        ros_pose.header.frame_id = "base_link";
    else
        ros_pose.header.frame_id = "map";
    ros_pose.pose.position.x = pose[0];
    ros_pose.pose.position.y = pose[1];
    ros_pose.pose.position.z = pose[2];
    return ros_pose;
}

float control::target_distance(Vector3f& target_pose){
    float dx = abs(mCurPoseROS.pose.position.x - target_pose[0]);
    float dy = abs(mCurPoseROS.pose.position.y - target_pose[1]);
    float dz = abs(mCurPoseROS.pose.position.z - target_pose[2]);
    return dx + dy + dz;
}

void control::move(Vector3f way_point, bool local_frame){
    if(local_frame){
        mMove_pub.publish(construct_pose(way_point, true));
    }
    else{
        mMove_pub.publish(construct_pose(way_point, false));
    }
}

bool control::fly_by_path(vector<Vector3f>& path){
    for(auto& way_point : path){
        if(target_distance(way_point) < 0.5)
            move(way_point, false);
    }
    return true;
}

// subscribers
void control::pose_sub(const geometry_msgs::PoseStamped &posemsg){
    mCurPoseROS = posemsg;
}

void control::pointcloud_sub(const sensor_msgs::PointCloud2& pointcloud) {
    mCurPointCloudROS = pointcloud;
    set_obstacle(mCurPointCloudROS);
}

void control::state_sub(const mavros_msgs::State& state) {
    mStateROS = state;
}

Vector3f control::continuous2discrete(Vector3f& continuous){
    Vector3f discrete;
    discrete[0] = int(continuous[0] + mGridSize * 0.5) / mGridSize - 1;
    discrete[1] = int(continuous[1] + mGridSize * 0.5) / mGridSize - 1;
    discrete[2] = int(continuous[2] + mGridSize * 0.5) / mGridSize - 1;
    return discrete;
}

Vector3f control::discrete2continuous(Vector3f& discrete){
    Vector3f continuous;
    continuous[0] = (discrete[0] + 0.5) * mGridSize;
    continuous[1] = (discrete[1] + 0.5) * mGridSize;
    continuous[2] = (discrete[2] + 0.5) * mGridSize;
    return continuous;
}

void control::set_obstacle(sensor_msgs::PointCloud2& ros_pointcloud2){

    sensor_msgs::PointCloud ros_pointcloud;
    sensor_msgs::convertPointCloud2ToPointCloud(ros_pointcloud2, ros_pointcloud);

    vector<Vector3f> pcVec;
//    cout<<"ros_pointcloud.points size: "<<ros_pointcloud.points.size()<<endl;
    for(auto& pt : ros_pointcloud.points){
        Vector3f continuous(pt.x, pt.y, pt.z);
//        cout<<"continuous: "<<continuous<<endl;
//        pcVec.emplace_back(continuous);
//        cout<<"continuous2discrete(continuous): "<<continuous2discrete(continuous)<<endl;
        pcVec.emplace_back(continuous2discrete(continuous));
    }
    mObstacleDiscrete = pcVec;
}