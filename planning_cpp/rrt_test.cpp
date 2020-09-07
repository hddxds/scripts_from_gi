//
// Created by gishr on 19-6-6.
//


#include "rrt_star.h"
#include "downsample.h"
#include "control.h"

int main(int argc, char **argv) {

//    rrt_star rrt(10);
//    vector<Vector3f> obstacle = rrt.create_obstacle();
//
//    Vector3f test_point(2, 0, 5);
//    bool not_hit = rrt.nodeCollisionCheck(Node(test_point));
//    cout<<"not hit? :"<<not_hit<<endl;
//
//    auto start = std::chrono::high_resolution_clock::now();
//    vector<Vector3f> path = rrt.find_path(Vector3f(0, 0, 0), Vector3f(40, 0, 5), obstacle);
//    auto finish = std::chrono::high_resolution_clock::now();
//    std::chrono::duration<double> elapsed = finish - start;
//    std::cout << "Time taken for finding path in second: " << elapsed.count() << " s"<<std::endl;
//
//    rrt.visualize_world();


    ros::init(argc, argv, "control");
    control test_control;
    test_control.start();
}