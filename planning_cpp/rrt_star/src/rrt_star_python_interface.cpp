//
// Created by gishr on 19-6-11.
//

#include "rrt_star.h"
#include <boost/python.hpp>


using namespace boost::python;


BOOST_PYTHON_MODULE(rrt)
{

//    rrt_star();
//
//    explicit rrt_star(float step_length, float target_sample_rate=40, int max_iter=10000);
//
//    inline void create_grid(Vector3f position);
//
//    vector<Vector3f> find_path(Vector3f start, Vector3f target, vector<Vector3f>& obstacle);
//
//    inline void set_obstacle(vector<Vector3f> obstacle);
//
//    inline Vector3f sample_from_grid();
//
//    int getNearestNodeIdx(Vector3f point);
//
//    Node move_one_step(Vector3f new_point, int nearest_node_idx);
//
//    bool nodeCollisionCheck(Node node, float safe_margin=5);
//
//    bool lineCollisionCheck(Node start, Node end, float safe_margin=5);
//
//    vector<int> findNearNodes(Node& new_node);
//
//    Node findSuitableParent(Node& old_node, Node& new_node, vector<int>& near_node_idxs);
//
//    int decide_last_node();
//
//    vector<Vector3f> get_path(int last_node_idx);
//
//    vector<Vector3f> create_obstacle();
//
//    void visualize_world();
//
//    float point_distance(Vector3f point1, Vector3f point2);
//
//    vector<cuboid> obstacle_cluster(vector<Vector3f>& points_list);

    class_<rrt_star>("rrt", init<float>(), init<float>(), init<int>())
            .def("find_path", &rrt_star::find_path)
            .def("set_obstacle", &rrt_star::set_obstacle)
            .def("create_obstacle", &rrt_star::create_obstacle)
            .def("visualize_world", &rrt_star::visualize_world)

}