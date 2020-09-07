//
// Created by gishr on 19-6-10.
//

#include "downsample.h"


downsample::downsample() {}

downsample::downsample(float resolution) {
    mResolution = resolution;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample::VoxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
        float leaf_size_x, float leaf_size_y, float leaf_size_z){

    cout<<"input_cloud size: "<<input_cloud->size()<<endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (input_cloud);
    sor.setLeafSize (leaf_size_x, leaf_size_y, leaf_size_z);
    sor.filter (*cloud_filtered);
    cout<<"filtered size: "<<cloud_filtered->size()<<endl;
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample::DownsampleWithOctreeAndGetVoxelCenters(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr){

    // step 1, create octree
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree (mResolution);

    // step 2, set clouds to octree
    octree.setInputCloud(input_cloud_ptr);
    octree.addPointsFromInputCloud ();

    // step 3, get voxel centroids
    //vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> voxel_centers;
    //pcl::octree::OctreePointCloud::AlignedPointTVector<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_centers (new pcl::PointCloud<pcl::PointXYZ> );
    int size = octree.getVoxelCentroids(voxel_centers->points);
    voxel_centers->width = voxel_centers->points.size ();
    voxel_centers->height = 1;
    voxel_centers->is_dense = true;
    cout<<"input_cloud size: "<<input_cloud_ptr->size()<<endl;
    cout<<"voxel center size: "<<size<<endl;

    return voxel_centers;
}

vector<Vector3f> downsample::DownsampleWithOctreeAndGetVoxelCenters(vector<Vector3f> obs){
    // step 1, create octree
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree (mResolution);

    // step 2, set clouds to octree
    auto input_cloud_ptr = VecEigen3fToPointCloud(obs);
    octree.setInputCloud(input_cloud_ptr);
    octree.addPointsFromInputCloud ();

    // step 3, get voxel centroids
    //vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> voxel_centers;
    //pcl::octree::OctreePointCloud::AlignedPointTVector<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_centers (new pcl::PointCloud<pcl::PointXYZ> );
    int size = octree.getVoxelCentroids(voxel_centers->points);

    vector<Vector3f> vec_centers;
    for(auto& pt : voxel_centers->points){
        vec_centers.emplace_back(pt.getVector3fMap());
    }

    return vec_centers;
}