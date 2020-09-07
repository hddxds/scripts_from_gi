//
// Created by gishr on 19-6-6.
//

#include <pcl_helper.h>
#include "rrt_star.h"

rrt_star::rrt_star(){};

rrt_star::rrt_star(float step_length, float target_sample_rate, int max_iter){
    mStepLength = step_length;
    mTargetSampleRate = target_sample_rate;
    mMaxIter = max_iter;

    mDownSamplePtr = make_shared<downsample>(0.5);
    mPointCloudClusterPtr = make_shared<clustering>(2, 1, 20000);
};


void rrt_star::create_grid(Vector3f position) {

    mGridMinX = int(position[0] - 1);
    mGridMaxX = int(position[0] + mGridX);
    mGridMinY = int(position[1] - mGridY/2);
    mGridMaxY = int(position[1] + mGridY/2);
    mGridMinZ = int(position[2] - mGridZ/2);
    mGridMaxZ = int(position[2] + mGridZ/2);

    cout<<"mGridMinX: "<<mGridMinX<<endl;
    cout<<"mGridMaxX: "<<mGridMaxX<<endl;
    cout<<"mGridMinY: "<<mGridMinY<<endl;
    cout<<"mGridMaxY: "<<mGridMaxY<<endl;
    cout<<"mGridMinZ: "<<mGridMinZ<<endl;
    cout<<"mGridMaxZ: "<<mGridMaxZ<<endl;

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
//    Node start_node(start);
//    start_node.cost = 0;
//    start_node.parent = -1;
    mNodeList.emplace_back(start);
    mSampledPoints.emplace_back(start);

    cout<<"find_path 3"<<endl;

    int iter=0;
    Node target_node(mTarget);
    cout<<"target_node.node_distance(Node(mNodeList.back()): "<<target_node.node_distance(Node(mNodeList.back()))<<endl;

    while(target_node.node_distance(Node(mNodeList.back())) > mStepLength && iter < mMaxIter){
//    while(target_node.node_distance(Node(mNodeList.back())) > mStepLength){
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
        cout<<"new_node nodeCollisionCheck(new_node)): "<<nodeCollisionCheck(new_node)<<endl;

        vector<int> near_node_idxs = findNearNodes(new_node);
        cout<<"near_node_idxs size: "<<near_node_idxs.size()<<endl;
        Node connected_node = findSuitableParent(mNodeList[nearest_node_idx], new_node, near_node_idxs);

        // if connected node has no parent, neglect this node and continue
        if(connected_node.parent != -1){
            mNodeList.emplace_back(connected_node);
        }
        else{
            continue;
        }
    }

    cout<<"find_path finished!"<<endl;

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
    cout<<"node_num: "<<node_num<<endl;
    for(int i=0; i<node_num; i++){
        cout<<distance_to_goal[i]<<", "<<mStepLength<<endl;
        if(distance_to_goal[i] <= mStepLength){
            qualified_node_indxs.emplace_back(i);
            cout<<"qualified id: "<<i<<endl;
        }
    }

    if(qualified_node_indxs.size() < 1){
        return -1;
    }

    int min_cost_idx = -1;
    float min_cost = std::numeric_limits<float>::max();
    for(auto& qualified_idx : qualified_node_indxs){
//        cout<<"qualified_idx: "<<qualified_idx<<endl;
//        cout<<"mNodeList[qualified_idx].cost: "<<mNodeList[qualified_idx].cost<<endl;
        if(mNodeList[qualified_idx].cost < min_cost){
            min_cost = mNodeList[qualified_idx].cost;
            min_cost_idx = qualified_idx;
        }
    }

    return min_cost_idx;
}

Node rrt_star::findSuitableParent(Node& old_node, Node& new_node, vector<int>& near_node_idxs){

    if (near_node_idxs.size() < 1){
        return old_node;
    }

    // 1. for each node around, calculate distance and cost
    vector<float> cost_list;
    for(auto& idx : near_node_idxs){
        Node potential_node = mNodeList[idx];
//        cout<<"potential_node: "<<potential_node.point()<<endl;
//        cout<<"new node: "<<new_node.point()<<endl;
        // float distance = potential_node.node_distance(new_node);
        float distance = potential_node.node_distance(new_node) + potential_node.cost;
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
    cout<<"min cost: "<<*ptr_min<<endl;
    if (*ptr_min == std::numeric_limits<float>::max()){
        return new_node;
    }

    int idx = std::distance(std::begin(cost_list), ptr_min);
    int near_node_idx = near_node_idxs[idx];
    cout<<"idx and near_node_idx: "<<idx<<", "<<near_node_idx<<endl;
    new_node.cost = *ptr_min;
    new_node.parent = near_node_idx;
    return new_node;
}


vector<int> rrt_star::findNearNodes(Node& new_node){
    int node_num = mNodeList.size();
    //float searching_radius = 50 * sqrt(log(node_num)/node_num);
    float searching_radius = 3;
    cout<<"node_num and searching_radius: "<<node_num<<", "<<searching_radius<<endl;
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
        cout<<"Testing obstacle cuboid!"<<endl;
        if (obstacle_cuboid.in_cuboid(point)){
            cout<<"point in cuboid!"<<endl;
            return false;
        }
        if (obstacle_cuboid.point_distance(point) < safe_margin){
            cout<<"point too close to cuboid!: "<<obstacle_cuboid.point_distance(point)<<endl;
            return false;
        } else{
            cout<<"obstacle_cuboid.point_distance(point): "<<obstacle_cuboid.point_distance(point)<<endl;
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
        new_node.x = int(nearest_node.x + (new_node.x - nearest_node.x) / mStepLength);
        new_node.y = int(nearest_node.y + (new_node.y - nearest_node.y) / mStepLength);
        new_node.z = int(nearest_node.z + (new_node.z - nearest_node.z) / mStepLength);
    }
    cout<<"new_node.x, y, z: "<<new_node.x<<", "<<new_node.y<<", "<<new_node.z<<endl;
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

    //NOTE downsample input obstacle points
    mObstacle3D = mDownSamplePtr->DownsampleWithOctreeAndGetVoxelCenters(obstacle);;
    vector<Vector3f> obstacle_current_height;

    for(auto& ob : obstacle){
        if (ob[2] == mStart[2]){
            obstacle_current_height.emplace_back(ob[0], ob[1], ob[2]);
        }
    }
    obstacle_cluster(mObstacle3D);
}

//// for test
//vector<Vector3f> rrt_star::create_obstacle() {
//    vector<Vector3f> obs;
//
//    for(int i=8; i<15; i++)
//        for(int j=-2; j<2 ;j++)
//            for(int k=-2; k<2; k++){
//                obs.emplace_back(i, j, k);
//            }
//
//    for(int i=25; i<30; i++)
//        for(int j=-2; j<2 ;j++)
//            for(int k=-2; k<2; k++){
//                obs.emplace_back(i, j, k);
//            }
//
//    for(auto& pt : obs){
//        cout<<"pt: "<<pt<<endl;
//    }
//
//    obstacle_cluster(obs);
//
//    return obs;
//}

// for test
vector<Vector3f> rrt_star::create_obstacle() {
    vector<Vector3f> obs;

    float step = 0.2;
    for(float i=8; i<15; ){
        i += step;
        for(float j=-6; j<3;){
            j += step;
            for(float k=-2; k<2;){
                k += step;
                obs.emplace_back(i, j, k);
            }
        }
    }

    for(float i=30; i<35; ){
        i += step;
        for(float j=5; j<10 ;){
            j += step;
            for(float k=-2; k<2;){
                k += step;
                obs.emplace_back(i, j, k);
            }
        }
    }

//    for(float i=45; i<50; ){
//        i += step;
//        for(float j=0; j<5;){
//            j += step;
//            for(float k=-2; k<2;){
//                k += step;
//                obs.emplace_back(i, j, k);
//            }
//        }
//    }

    for(auto& pt : obs){
        cout<<"pt: "<<pt<<endl;
    }

    obs = mDownSamplePtr->DownsampleWithOctreeAndGetVoxelCenters(obs);
    obstacle_cluster(obs);

    return obs;
}

vector<cuboid> rrt_star::obstacle_cluster(vector<Vector3f>& points_list) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr = VecEigen3fToPointCloud(points_list);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr = mDownSamplePtr->DownsampleWithOctreeAndGetVoxelCenters(input_cloud_ptr);

    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec_cluster_ptrs = mPointCloudClusterPtr->EuclideanClusterExtraction(input_cloud_ptr, true);

    vector<cuboid> cuboids;
    // cuboids.resize(vec_cluster_ptrs.size());

    for(auto& pc_ptr : vec_cluster_ptrs){
        auto vec_3f = PointCloudToVec3f(pc_ptr);
        cuboid cube(vec_3f);
        cuboids.emplace_back(cube);
    }

    cout<<"Total number of points: "<<vec_cluster_ptrs.size()<<endl;
    cout<<"obstacle_cluster, clustered cuboid number: "<<cuboids.size()<<endl;
    mObstacleCuboidLists = cuboids;
    mObstacle3D = points_list;

    return cuboids;
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
    for(auto& p : mPath){
        pxs.emplace_back(p[0]);
        pys.emplace_back(p[1]);
        pzs.emplace_back(p[2]);
    }

//    vector<Vector3f> in_cuboid_points;
//    for(auto& pt : mObstacle3D){
//        if (not nodeCollisionCheck(Node(pt))){
//            in_cuboid_points.emplace_back(pt);
//        }
//    }

    vector<double> oxs, oys, ozs;
    for(auto& p : mObstacle3D){
        oxs.emplace_back(p[0]);
        oys.emplace_back(p[1]);
        ozs.emplace_back(p[2]);
    }

//    vector<float> cx, cy, cz;
//    for(auto& cuboid : mObstacleCuboidLists){
//        for(auto& corner : cuboid.mVertices){
//            cx.emplace_back(corner[0]);
//            cy.emplace_back(corner[1]);
//            cz.emplace_back(corner[2]);
//        }
//    }

    vector<float> cx0, cy0, cz0;
    for(auto& cuboid : mObstacleCuboidLists){
        cx0.emplace_back(cuboid.mVertices[0][0]);
        cy0.emplace_back(cuboid.mVertices[0][1]);
        cz0.emplace_back(cuboid.mVertices[0][2]);
    }

    vector<float> cx7, cy7, cz7;
    for(auto& cuboid : mObstacleCuboidLists){
        cx7.emplace_back(cuboid.mVertices[7][0]);
        cy7.emplace_back(cuboid.mVertices[7][1]);
        cz7.emplace_back(cuboid.mVertices[7][2]);
    }

    vector<float> cx3, cy3, cz3;
    for(auto& cuboid : mObstacleCuboidLists){
        cx3.emplace_back(cuboid.mVertices[3][0]);
        cy3.emplace_back(cuboid.mVertices[3][1]);
        cz3.emplace_back(cuboid.mVertices[3][2]);
    }

    vector<float> cx5, cy5, cz5;
    for(auto& cuboid : mObstacleCuboidLists){
        cx5.emplace_back(cuboid.mVertices[7][0]);
        cy5.emplace_back(cuboid.mVertices[7][1]);
        cz5.emplace_back(cuboid.mVertices[7][2]);
    }




    //draw all scatter plot here
    // 1. create figure
    PyObject *fig = PyObject_CallObject(matplotlibcpp::detail::_interpreter::get().s_python_function_figure,
                                        matplotlibcpp::detail::_interpreter::get().s_python_empty_tuple);
    // 2. sampled point
    fig = matplotlibcpp::scatter(fig, xs, ys, zs, 1, "r");
    // 3. path
    if(pxs.size() > 0)
        fig = matplotlibcpp::scatter(fig, pxs, pys, pzs, 20, "b");
    // 4. obstacle
    fig = matplotlibcpp::scatter(fig, oxs, oys, ozs, 5, "g");
    // 5. cuboid corners
    fig = matplotlibcpp::scatter(fig, cx0, cy0, cz0, 100, "k");
    fig = matplotlibcpp::scatter(fig, cx7, cy7, cz7, 100, "y");

    matplotlibcpp::show();
}


