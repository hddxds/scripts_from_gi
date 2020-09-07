//
// Created by gishr on 19-6-6.
//

#ifndef CUBOID_H
#define CUBOID_H


#include <cmath>
#include <Eigen/Core>
#include <iostream>
#include <vector>
#include <limits>

using namespace std;
using namespace Eigen;

class cuboid{

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

    cuboid();

    explicit cuboid(vector<Vector3f>& points);

    bool in_cuboid(Vector3f& point);

    float point_distance(Vector3f& point);

    float cuboid_distance(cuboid& cube);


public:

    float mMin_x, mMax_x;
    float mMin_y, mMax_y;
    float mMin_z, mMax_z;
    Vector3f mCenter;
    vector<Vector3f> mVertices;
    vector<Vector3f> mObstacle3D;
    vector<Vector3f> mObstacleCurrentHeight;
};


#endif //CUBOID_H
