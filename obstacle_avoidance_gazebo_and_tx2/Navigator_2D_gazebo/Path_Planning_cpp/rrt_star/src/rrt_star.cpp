//
// Created by gishr on 19-6-6.
//

#include "rrt_star.h"

rrt_star::rrt_star(){};

rrt_star::rrt_star(float step_length, float target_sample_rate, int max_iter){
    mStepLength = step_length;
    mTargetSampleRate = target_sample_rate;
    mMaxIter = max_iter;

};


void rrt_star::create_grid(Vector3f position) {

    mGridMinX = position[0] - 1;
    mGridMaxX = position[0] + mGridX;
    mGridMinY = position[1] - int(mGridY/2);
    mGridMaxY = position[1] + int(mGridY/2);
    mGridMinZ = position[2] - int(mGridZ/2);
    mGridMaxZ = position[2] + int(mGridZ/2);

    std::uniform_int_distribution<int> uni_dist_x(mGridMinX, mGridMaxX);
    std::uniform_int_distribution<int> uni_dist_y(mGridMinY, mGridMaxY);
    std::uniform_int_distribution<int> uni_dist_z(mGridMinZ, mGridMaxZ);
    std::uniform_int_distribution<int> uni_dist_p(0, 100);
    mRandomX = uni_dist_x;
    mRandomY = uni_dist_y;
    mRandomZ = uni_dist_z;
    mRandomP = uni_dist_p;
}



vector<Vector3f> rrt_star::find_path(Vector3f start, Vector3f target, vector<Vector3f>& obstacle) {

    cout<<"find_path 1"<<endl;

    mStart = start;
    mTarget = target;
    set_obstacle(obstacle);
    create_grid(start);

    cout<<"find_path 2"<<endl;

    mNodeList.emplace_back(start);
    mSampledPoints.emplace_back(start);

    cout<<"find_path 3"<<endl;

    int iter=0;
    Node start_node(mStart);
    while(start_node.node_distance(Node(mNodeList.back())) < mStepLength && iter < mMaxIter){
        cout<<"find_path iter:"<<iter<<endl;
        iter ++;
        Vector3f sampled_pt = sample_from_grid();
        mSampledPoints.emplace_back(sampled_pt);
        cout<<"sampled point: "<<sampled_pt[0]<<", "<<sampled_pt[1]<<", "<<sampled_pt[2]<<endl;

        int nearest_node_idx = getNearestNodeIdx(sampled_pt);
        cout<<"Nearest Node indx: "<<nearest_node_idx<<endl;

        Node new_node = move_one_step(sampled_pt, nearest_node_idx);

        if (!nodeCollisionCheck(new_node)){
            continue;
        }

        cout<<"nodeCollisionCheck(new_node)): "<<nodeCollisionCheck(new_node)<<endl;

        vector<int> near_node_idxs = findNearNodes(new_node);
        cout<<"near_node_idxs size: "<<near_node_idxs.size()<<endl;

        Node connected_node = findSuitableParent(new_node, near_node_idxs);

        // if connected node has no parent, neglect this node and continue
        if(connected_node.parent != -1){
            mNodeList.emplace_back(connected_node);
        }
        else{
            continue;
        }
    }

    int last_node_idx = decide_last_node();
    if(last_node_idx == -1){
        cout<<"Finding path failed!"<<endl;
        vector<Vector3f> None_path;
        return None_path;
    }

    cout<<"Path found!"<<endl;
    return get_path(last_node_idx);
}


vector<Vector3f> rrt_star::get_path(int node_idx){
    vector<Vector3f> path;
    path.emplace_back(mTarget);

    int last_node_idx = node_idx;
    while(mNodeList[last_node_idx].parent != -1){
        Node last_node = mNodeList[last_node_idx];
        path.emplace_back(last_node.point());
        last_node_idx = last_node.parent;
    }

    path.emplace_back(mStart);

    // reverse the path
    std::reverse(path.begin(),path.end());
    mPath = path;
    return path;
}

int rrt_star::decide_last_node() {
    vector<float> distance_to_goal;
    vector<int> qualified_node_indxs;
    for(auto& node : mNodeList){
        cout<<"point_distance(node.point(), mTarget): "<<point_distance(node.point(), mTarget)<<endl;
        distance_to_goal.emplace_back(point_distance(node.point(), mTarget));
    }

    int node_num = mNodeList.size();
    for(int i; i<node_num; i++){
        if(distance_to_goal[i] < mStepLength){
            qualified_node_indxs.emplace_back(i);
        }
    }

    if(qualified_node_indxs.size() < 1){
        return -1;
    }

    int min_cost_idx = -1;
    float min_cost = std::numeric_limits<float>::max();
    for(auto& qualified_idx : qualified_node_indxs){
        if(mNodeList[qualified_idx].cost < min_cost){
            min_cost = mNodeList[qualified_idx].cost;
            min_cost_idx = qualified_idx;
        }
    }

    return min_cost_idx;
}

Node rrt_star::findSuitableParent(Node& new_node, vector<int>& near_node_idxs){

    if (near_node_idxs.size() < 1){
        return new_node;
    }

    // 1. for each node around, calculate distance and cost
    vector<float> cost_list;
    for(auto& idx : near_node_idxs){
        Node potential_node = mNodeList[idx];
        float distance = potential_node.node_distance(new_node);
        bool not_collision = lineCollisionCheck(new_node, potential_node);
        if(not_collision){
            cost_list.emplace_back(potential_node.cost + distance);
        }
        else{
            cost_list.emplace_back(std::numeric_limits<float>::max());
        }
    }

    // 2. find min cost in cost_list and
    auto ptr_min = std::min_element(std::begin(cost_list), std::end(cost_list));
    if (*ptr_min == std::numeric_limits<float>::max()){
        return new_node;
    }

    int idx = std::distance(std::begin(cost_list), ptr_min);
    int near_node_idx = near_node_idxs[idx];

    new_node.cost = *ptr_min;
    new_node.parent = near_node_idx;
    return new_node;
}


vector<int> rrt_star::findNearNodes(Node& new_node){
    int node_num = mNodeList.size();
    float searching_radius = 50 * sqrt(log(node_num)/node_num);
    vector<int> near_node_idxs;
    for(int i=0; i<node_num; i++){
        if (new_node.node_distance(mNodeList[i]) <= searching_radius*searching_radius){
            near_node_idxs.emplace_back(i);
        }
    }
    return near_node_idxs;
}


bool rrt_star::lineCollisionCheck(Node start, Node end, float safe_margin){
    vector<Vector3f> line = Bresenham3D(start.point(), end.point());
    for(auto& obstacle_cuboid : mObstacleCuboidLists){
        for(auto& pt : line){
            if (obstacle_cuboid.in_cuboid(pt)){
                return false;
            }
            if (obstacle_cuboid.point_distance(pt) < safe_margin){
                return false;
            }
        }
    }
    return true;
}

bool rrt_star::nodeCollisionCheck(Node node, float safe_margin){
    Vector3f point = node.point();
    for(auto obstacle_cuboid : mObstacleCuboidLists){
        if (obstacle_cuboid.in_cuboid(point)){
            return false;
        }
        if (obstacle_cuboid.point_distance(point) < safe_margin){
            return false;
        }
    }
    return true;
}

Node rrt_star::move_one_step(Vector3f new_point, int nearest_node_idx) {

    Node nearest_node = mNodeList[nearest_node_idx];
    Node new_node(new_point);
    float node_dist = new_node.node_distance(nearest_node);
    cout<<"node_dist : "<<node_dist<<endl;
    if (node_dist < mStepLength){
        return nearest_node;
    }
    else{
        new_node.x = nearest_node.x + (new_node.x - nearest_node.x) / mStepLength;
        new_node.y = nearest_node.y + (new_node.y - nearest_node.y) / mStepLength;
        new_node.z = nearest_node.z + (new_node.z - nearest_node.z) / mStepLength;
    }
    cout<<"new_node.x, y, z: "<<new_node.x<<", "<<new_node.y<<", "<<new_node.z<<endl;
    cout<<"New node position: "<<new_node.point()<<endl;
    new_node.cost = std::numeric_limits<float>::max();
    new_node.parent = -1;

    return new_node;
}

int rrt_star::getNearestNodeIdx(Vector3f point){
    vector<int> vec_idx;
    int node_num = mNodeList.size();
    cout<<"node num: "<<node_num<<endl;
    int min_indx;
    float min_distance = std::numeric_limits<float>::max();
    for (int i=0; i<node_num; i++){
        float pt_distance = point_distance(point, mNodeList[i].point());
        if (pt_distance < min_distance){
            min_distance = pt_distance;
            min_indx = i;
        }
    }

    return min_indx;
}

float rrt_star::point_distance(Vector3f point1, Vector3f point2) {
    float dx = (point1[0] - point2[0])*(point1[0] - point2[0]);
    float dy = (point1[1] - point2[1])*(point1[1] - point2[1]);
    float dz = (point1[2] - point2[2])*(point1[2] - point2[2]);
    return sqrt(dx+dy+dz);
}

Vector3f rrt_star::sample_from_grid() {
    if (mRandomP(mRandomEngine) > mTargetSampleRate){
        return Vector3f(mRandomX(mRandomEngine), mRandomY(mRandomEngine), mRandomZ(mRandomEngine));
    }
    else{
        return Vector3f(mTarget[0], mTarget[1], mTarget[2]);
    }
}

void rrt_star::set_obstacle(vector<Vector3f> obstacle){
    mObstacle3D = obstacle;
    vector<Vector3f> obstacle_current_height;

    for(auto& ob : obstacle){
        if (ob[2] == mStart[2]){
            obstacle_current_height.emplace_back(ob[0], ob[1], ob[2]);
        }
    }
}

vector<Vector3f> rrt_star::create_obstacle() {
    vector<Vector3f> obs;
    for(int i=0; i<5; i++)
        for(int j=-10; j<10 ;j++)
            for(int k=0; k<20; k++){
                obs.emplace_back(i, j, k);
            }
    return obs;
}

void rrt_star::visualize_world() {

    // sampled points
    vector<double> xs, ys, zs;
    for(auto& p : mSampledPoints){
        xs.emplace_back(p[0]);
        ys.emplace_back(p[1]);
        zs.emplace_back(p[2]);
    }

    vector<double> pxs, pys, pzs;
    for(auto& p : mObstacle3D){
        cout<<"p: "<<p[0]<<", "<<p[1]<<", "<<p[2]<<endl;
        pxs.emplace_back(p[0]);
        pys.emplace_back(p[1]);
        pzs.emplace_back(p[2]);
    }



    plt::scatter(xs, ys, zs, 1, "r");
    plt::scatter(pxs, pys, pzs, 10, "g");

    plt::show();

}


