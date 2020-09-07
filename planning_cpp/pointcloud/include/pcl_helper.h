//
// Created by shr on 19-6-9.
//

#ifndef PCL_HELPER_H
#define PCL_HELPER_H


#include "core.h"

using namespace Eigen;
using namespace std;

// convert Eigen::Vector3f to PCL pointXYZ
pcl::PointXYZ Point3fToPCLXYZ(Vector3f eigen_point);

// convert vector<Eigen::Vector3f> to PCL pointcloud<PointXYZ>::ptr
pcl::PointCloud<pcl::PointXYZ>::Ptr VecEigen3fToPointCloud(vector<Vector3f>& points);

void SavePointCloudToPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);

vector<Vector3f> PointCloudToVec3f(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_ptr);


#endif //PCL_HELPER_H
