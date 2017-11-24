//
//  GetBoundaryOriginal.hpp
//  PoinCloudDebug
//
//  Created by Jonathan闫 on 2017/11/15.
//  Copyright © 2017年 Jonathan闫. All rights reserved.
//

#ifndef GetBoundaryOriginal_hpp
#define GetBoundaryOriginal_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>  

pcl::PointCloud<pcl::Boundary> getBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int kSearch, double angleThreshold);

pcl::PointCloud<pcl::PointXYZ>::Ptr extractBoundary(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    pcl::PointCloud<pcl::Boundary> &boundary);

#endif /* GetBoundaryOriginal_hpp */
