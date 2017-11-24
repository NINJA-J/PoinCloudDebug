//
//  Filters.hpp
//  PoinCloudDebug
//
//  Created by Jonathan闫 on 2017/11/21.
//  Copyright © 2017年 Jonathan闫. All rights reserved.
//

#ifndef Filters_hpp
#define Filters_hpp

#include <stdio.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/impl/bilateral.hpp>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/impl/point_types.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/boundary.h>
#include <pcl/filters/convolution_3d.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr bilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double r,double s);

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double lx, double ly, double lz);

pcl::PointCloud<pcl::PointXYZ>::Ptr gaussianKernelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, double sigma, double threshold);

#endif /* Filters_hpp */
