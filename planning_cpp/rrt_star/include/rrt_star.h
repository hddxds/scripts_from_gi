//
// Created by gishr on 19-6-6.
//

#ifndef RRT_STAR_H
#define RRT_STAR_H

#include "cuboid.h"
#include "Node.h"
#include "bresenham.h"
#include "matplotlibcpp.h"
#include "clustering.h"
#include "downsample.h"
#include "cmath"
#include <Eigen/Core>
#include <iostream>
#include <random>
#include <algorithm>
#include <chrono>
#include <memory>

using namespace Eigen;
using namespace std;

class rrt_star{

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

    rrt_star();

    explicit rrt_star(float step_length, float target_sample_rate=40, int max_iter=10000);

    inline void create_grid(Vector3f position);

    vector<Vector3f> find_path(Vector3f start, Vector3f target, vector<Vector3f>& obstacle);

    inline void set_obstacle(vector<Vector3f> obstacle);

    inline Vector3f sample_from_grid();

    int getNearestNodeIdx(Vector3f point);

    Node move_one_step(Vector3f new_point, int nearest_node_idx);

    bool nodeCollisionCheck(Node node, float safe_margin=5);

    bool lineCollisionCheck(Node start, Node end, float safe_margin=5);

    vector<int> findNearNodes(Node& new_node);

    Node findSuitableParent(Node& old_node, Node& new_node, vector<int>& near_node_idxs);

    int decide_last_node();

    vector<Vector3f> get_path(int last_node_idx);

    vector<Vector3f> create_obstacle();

    void visualize_world();

    float point_distance(Vector3f point1, Vector3f point2);

    vector<cuboid> obstacle_cluster(vector<Vector3f>& points_list);

private:

    shared_ptr<clustering> mPointCloudClusterPtr = nullptr;
    shared_ptr<downsample> mDownSamplePtr = nullptr;

    float mStepLength;
    float mTargetSampleRate;
    int mMaxIter;

    int mGridX=80, mGridY=40, mGridZ=20;
    int mGridMinX, mGridMaxX, mGridMinY, mGridMaxY, mGridMinZ, mGridMaxZ;

    vector<Vector3f> mPath;

    std::default_random_engine mRandomEngine;
    std::uniform_int_distribution<int> mRandomX, mRandomY, mRandomZ, mRandomP;

    vector<cuboid> mObstacleCuboidLists;

public:
    Vector3f mStart, mTarget;
    vector<Vector3f> mObstacle3D, mObstacleCurrentHeight;
    vector<Node> mNodeList;
    vector<Vector3f> mSampledPoints;

};




#endif //RRT_STAR_H
