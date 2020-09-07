//
// Created by shr on 19-6-9.
//

#ifndef CLUSTERING_H
#define CLUSTERING_H

#include "core.h"

using namespace Eigen;
using namespace std;

class clustering {

public:

    clustering();

    clustering(float tolerance=2.0, int min_num=10, int max_num=2000);

    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> EuclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud, bool write_results=true);

private:
    int mMinPointNum, mMaxPointNum;
    float mTolerance;

    pcl::PCDWriter* mPclWriter = nullptr;
};


#endif //CLUSTERING_H
