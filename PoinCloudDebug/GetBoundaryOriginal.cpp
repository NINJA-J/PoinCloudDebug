//
//  GetBoundaryOriginal.cpp
//  PoinCloudDebug
//
//  Created by Jonathan闫 on 2017/11/15.
//  Copyright © 2017年 Jonathan闫. All rights reserved.
//

#include "GetBoundaryOriginal.hpp"

using namespace std;

pcl::PointCloud<pcl::Boundary> getBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int kSearch, double angleThreshold){
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    normEst.setInputCloud(cloud);
//    normEst.setRadiusSearch(radius);
    normEst.setKSearch(10);
    normEst.compute(*normals);
    
    cout<<"Normal Estimation Builded"<<endl;
    
    boundEst.setInputCloud(cloud);
    boundEst.setInputNormals(normals);
//    boundEst.setRadiusSearch(radius);
    boundEst.setAngleThreshold(angleThreshold);
    boundEst.setSearchMethod(tree);
    boundEst.setKSearch(kSearch);
    boundEst.compute(boundaries);
    
    cout<<"Boundary Estimation Builded"<<endl;
    
    return boundaries;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extractBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Boundary> &boundary){
    pcl::PointCloud<pcl::PointXYZ>::Ptr bound(new pcl::PointCloud<pcl::PointXYZ>());
    for(int i=0;i<boundary.points.size();i++){
        if(static_cast<int>(boundary.points[i].boundary_point)!=0)
            bound->points.push_back(cloud->points[i]);
    }
    bound->width = bound->points.size();
    bound->height = 1;
    double percentage = (double)boundary.points.size()/(double)cloud->points.size()*100;
    int bPoints = bound->width;
    int oPoints = cloud->points.size();
    cout<<bPoints<<"/"<<oPoints<<" Boundary Points Extracted From Original Cloud"<<endl;
    return bound;
}
