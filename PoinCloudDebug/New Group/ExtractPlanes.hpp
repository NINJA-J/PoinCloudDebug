//
//  ExtractPlanes.hpp
//  PoinCloudDebug
//
//  Created by Jonathan闫 on 2017/11/14.
//  Copyright © 2017年 Jonathan闫. All rights reserved.
//

#ifndef ExtractPlanes_hpp
#define ExtractPlanes_hpp

#include <stdio.h>

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <string>

#include <vector>

using namespace std;

class ExtractInfo{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr pIndices;
    pcl::ModelCoefficients::Ptr pCoef;
    
    ExtractInfo();
//    ExtraceInfo(
//                pcl::PointCloud<pcl::PointXYZ>::Ptr pIndices,
//                pcl::ModelCoefficients::Ptr pCoef);
};

vector<ExtractInfo> extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,double remains = 0.1, int maxPlanes = 5, double dt = 0.0001);

ExtractInfo extractPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double dt = 0.0001);

void saveExtractPlanes( vector<ExtractInfo> &infos, string groupName = "Extract Plane", string ending = "pcd" );

#endif /* ExtractPlanes_hpp */
