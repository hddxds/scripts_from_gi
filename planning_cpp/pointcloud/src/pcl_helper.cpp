//
// Created by shr on 19-6-9.
//

#include "pcl_helper.h"



pcl::PointXYZ Point3fToPCLXYZ(Vector3f eigen_point){
    pcl::PointXYZ pt;
    pt.getVector3fMap() = eigen_point;
    return pt;
}

// convert vector<Eigen::Vector3f> to PCL pointcloud<PointXYZ>::ptr
pcl::PointCloud<pcl::PointXYZ>::Ptr VecEigen3fToPointCloud(vector<Vector3f>& points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto &pt : points) {
        auto ptxyz = Point3fToPCLXYZ(pt);
        pc->points.push_back(ptxyz);
    }
    pc->width = pc->points.size ();
    pc->height = 1;
    pc->is_dense = true;

    return pc;
}

void SavePointCloudToPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud){
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("octreecenters.pcd", *pointcloud, false);
}


vector<Vector3f> PointCloudToVec3f(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_ptr){
    vector<Vector3f> vec_3f;
    for(auto& aligned_pt : pc_ptr->points){
        vec_3f.emplace_back(aligned_pt.getVector3fMap());
    }
    return vec_3f;
}