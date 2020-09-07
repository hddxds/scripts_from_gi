//
// Created by shr on 19-6-9.
//

#include "clustering.h"
#include "pcl_helper.h"


clustering::clustering() {};

clustering::clustering(float tolerance, int min_num, int max_num) {
    mTolerance = tolerance;
    mMinPointNum = min_num;
    mMaxPointNum = max_num;
    cout<<"mTolerance: "<<mTolerance<<", mMinPointNum: "<<mMinPointNum<<", mMaxPointNum: "<<mMaxPointNum<<endl;
}


vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustering::EuclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud, bool write_results){

    // auto raw_cloud = VecEigen3fToPointCloud(input_pc);

    // step 1, create KD tree and set input
    pcl::search::KdTree<pcl::PointXYZ>::Ptr mKdTree (new pcl::search::KdTree<pcl::PointXYZ>);
    mKdTree->setInputCloud (raw_cloud);

    // step 2, create ECE instance and conduct Extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (mTolerance); // 2cm
    ec.setMinClusterSize (mMinPointNum);
    ec.setMaxClusterSize (mMaxPointNum);
    ec.setSearchMethod (mKdTree);
    ec.setInputCloud (raw_cloud);
    ec.extract (cluster_indices);

//    for(auto pt: raw_cloud->points){
//        cout<<"pt: "<<pt<<endl;
//    }
//
//    for(int i=0; i<raw_cloud->size(); i++)
//    {
//        cout<<"i and pt: "<<i<<", "<<raw_cloud->points[i]<<endl;
//    }

    // step 3, create pointcloud and assign pointXYZ to the each cloud
    int number_of_clusters = 0;
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_ptrs;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (auto pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
            cloud_cluster->points.push_back(raw_cloud->points[*pit]);
        }

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cluster_ptrs.push_back(cloud_cluster);

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << number_of_clusters << ".pcd";

        if(write_results){
            mPclWriter->write<pcl::PointXYZ>(ss.str (), *cloud_cluster, false);
        }
        number_of_clusters++;
    }
    cout<<"Total "<<raw_cloud->points.size()<<" points!"<<endl;
    cout<<"Total "<<number_of_clusters<<" clusters found!"<<endl;
    return cluster_ptrs;
}