//
// Created by gishr on 19-6-10.
//

#ifndef DOWNSAMPLE_H
#define DOWNSAMPLE_H

#include "pcl_helper.h"
#include "core.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

using namespace std;
using namespace Eigen;

class downsample {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    downsample();

    downsample(float resolution);

    pcl::PointCloud<pcl::PointXYZ>::Ptr VoxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
            float leaf_size_x = 0.1, float leaf_size_y = 0.1, float leaf_size_z = 0.1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr DownsampleWithOctreeAndGetVoxelCenters(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr);

    vector<Vector3f> DownsampleWithOctreeAndGetVoxelCenters(vector<Vector3f> obs);

private:
    float mResolution = 0;


};


#endif //DOWNSAMPLE_H
