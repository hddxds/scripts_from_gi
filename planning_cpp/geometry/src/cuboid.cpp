//
// Created by gishr on 19-6-6.
//

#include "cuboid.h"


cuboid::cuboid() = default;

cuboid::cuboid(vector<Vector3f>& points){

    float min_x=std::numeric_limits<float>::max();
    float min_y=std::numeric_limits<float>::max();
    float min_z=std::numeric_limits<float>::max();
    float max_x=0, max_y=0, max_z=0;

    for(auto& point : points){
        if (point[0] < min_x)
            min_x = point[0];
        if (point[0] > max_x)
            max_x = point[0];
        if (point[1] < min_y)
            min_y = point[1];
        if (point[1] > max_y)
            max_y = point[1];
        if (point[2] < min_z)
            min_z = point[2];
        if (point[2] > max_z)
            max_z = point[2];
    }

    mMin_x = min_x; mMax_x = max_x; mMin_y = min_y;
    mMax_y = max_y; mMin_z = min_z; mMax_z = max_z;

    mCenter[0] = (mMin_x + mMax_x)/2.0;
    mCenter[1] = (mMin_y + mMax_y)/2.0;
    mCenter[2] = (mMin_z + mMax_z)/2.0;

    /*
     p8                p7
       /---------------/|
    p5/|____________p6/ |
     | |             |  |
     | /p4-----------|-/ p3
     |/______________|/
     p1              p2
    */
    mVertices.emplace_back(mMin_x, mMin_y, mMin_z);
    mVertices.emplace_back(mMin_x, mMax_y, mMin_z);
    mVertices.emplace_back(mMin_x, mMin_y, mMax_z);
    mVertices.emplace_back(mMin_x, mMax_y, mMax_z);
    mVertices.emplace_back(mMax_x, mMin_y, mMin_z);
    mVertices.emplace_back(mMax_x, mMax_y, mMin_z);
    mVertices.emplace_back(mMax_x, mMin_y, mMax_z);
    mVertices.emplace_back(mMax_x, mMax_y, mMax_z);

    for(auto& corner : mVertices){
        cout<<"cuboid corner: "<<corner<<endl;
    }

}

bool cuboid::in_cuboid(Vector3f& point) {
    if (point[0] < mMin_x || point[0] > mMax_x)
        return false;
    if (point[1] < mMin_y || point[1] > mMax_y)
        return false;
    if (point[2] < mMin_z || point[2] > mMax_z)
        return false;

    return true;
}

float cuboid::point_distance(Vector3f& point) {
    if (in_cuboid(point)){
        return -1;
    }
    else{
        float dx = max(max(mMin_x-point[0], float(0.0)), point[0]-mMax_x);
        float dy = max(max(mMin_y-point[1], float(0.0)), point[1]-mMax_y);
        float dz = max(max(mMin_z-point[2], float(0.0)), point[2]-mMax_z);
        return sqrt(dx*dx + dy*dy + dz*dz);
    }
}

float cuboid::cuboid_distance(cuboid& cube) {
    float max_distance=std::numeric_limits<float>::max();
    float point_distance = 0.0;
    for(auto& vertex_1 : mVertices){
        if (in_cuboid(vertex_1))
            return 0;
        for(auto& vertex_2 : cube.mVertices){
            point_distance = sqrt((vertex_1[0]-vertex_2[0])*(vertex_1[0]-vertex_2[0])
                    + (vertex_1[0]-vertex_2[0])*(vertex_1[0]-vertex_2[0])
                    + (vertex_1[0]-vertex_2[0])*(vertex_1[0]-vertex_2[0]));
            if (point_distance < max_distance)
                point_distance = max_distance;
        }
    }
    return point_distance;
}

//vector<Vector3f> cuboid::load_obstacle(string &file_path) {
//
//}

