//
// Created by gishr on 19-6-6.
//

#ifndef NODE_H
#define NODE_H

#include <Eigen/Core>
#include <cmath>

using namespace Eigen;

class Node{

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

    Node();

    explicit Node(Vector3f pt){
        x = pt[0];
        y = pt[1];
        z = pt[2];
    }


    inline float node_distance(Node node){
        float dx = (x - node.x) * (x - node.x);
        float dy = (y - node.y) * (y - node.y);
        float dz = (z - node.z) * (z - node.z);
        return sqrt(dx+dy+dz);
    }

    inline Vector3f point(){
        return Vector3f(x, y, z);
    }

public:

    float x, y, z;
    int parent = -1;
    float cost = 0;
};




#endif //NODE_H
