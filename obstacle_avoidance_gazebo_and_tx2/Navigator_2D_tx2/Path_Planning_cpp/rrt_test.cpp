//
// Created by gishr on 19-6-6.
//


#include "rrt_star.h"

int main(int argc, char **argv) {

    rrt_star rrt(50);
    vector<Vector3f> obstacle = rrt.create_obstacle();
    for(auto& ob : obstacle)
    {
        cout<<ob<<endl;
    }

    vector<Vector3f> path = rrt.find_path(Vector3f(0, 0, 10), Vector3f(50, 0, 10), obstacle);
    rrt.visualize_world();


//    vector<Vector3f> line = Bresenham3D(Vector3f(0, 0, 0), Vector3f(5, 6, 7));
//    for(auto& p : line){
//        cout<<"line p: "<<p[0]<<", "<<p[1]<<", "<<p[2]<<endl;
//    }

//    vector<double> xs, ys, zs;
//    for(auto& p : line){
//        cout<<"p: "<<p[0]<<", "<<p[1]<<", "<<p[2]<<endl;
//        xs.emplace_back(p[0]);
//        ys.emplace_back(p[1]);
//        zs.emplace_back(p[2]);
//    }

//    plt::scatter(xs, ys, zs, 50, "r");
//    plt::show();

}