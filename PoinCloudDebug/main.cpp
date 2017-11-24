//
//  main.cpp
//  PoinCloudDebug
//
//  Created by Jonathan闫 on 2017/11/6.
//  Copyright © 2017年 Jonathan闫. All rights reserved.
//

// 离散化处理 ：
//      xDis = 3.78202  84.0258 - 80.2438
//      yDis = 3.685    101.974 - 98.2886
//      zDis = 1.77634  4.88225 - 3.10591

//http://blog.csdn.net/jiaojialulu/article/details/69351034

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/boundary.h>
#include "CreatePCDFromTxt.hpp"
#include "GetBoundaryOriginal.hpp"
#include "ExtractPlanes.hpp"
#include "Filters.hpp"

using namespace std;

void savePCDFile(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, string dir, string fName){
    pcl::io::savePCDFile( "./" + dir + "/PCD/" + fName + ".pcd", *cloud, true);
    ofstream fout(        "./" + dir + "/TXT/" + fName + ".txt" );
    vector<pcl::PointXYZ>::iterator iter = cloud->points.begin();
    while( iter != cloud->points.end() ){
        fout << iter->x << " " << iter->y << " " << iter->z << endl;
        iter ++;
    }
    cout << "File " + fName + " PCD & TXT Mode Saved In " + dir + " Folder" << endl;
}

void savePCDFile(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, string fName){
    savePCDFile(cloud, "Origin", fName);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr myPc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr VGF_Cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr GF_Cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr VGF_GF_Cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr SubCloud(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointIndices::Ptr inliers( new pcl::PointIndices );

int main(int argc, const char * argv[]) {
    double xMin = 80.2438;
    double xMax = 82.6;
    double yMin = 98.2886;
    double yMax = 100.48;
    
    pcl::io::loadPCDFile("Origin/PCD/ArchOrigin.pcd", *myPc);
    cout << myPc->points.size() << " Points Loaded" <<endl;
    
    vector<pcl::PointXYZ>::iterator iter = myPc->points.begin();
    while( iter != myPc->points.end() ){
        if( xMin <= iter->x && iter->x <= xMax && yMin <= iter->y && iter->y <= yMax )
            SubCloud->points.push_back(*iter);
        iter++;
    }
    SubCloud->width = SubCloud->points.size();
    SubCloud->height = 1;
    cout << "Extract " << SubCloud->width << " Points From The Box" << endl;
    savePCDFile( SubCloud, "SubPart1", "Origin" );
//    cout << "Source PCD File Loaded" << endl;
//
//    VGF_Cloud = voxelGridFilter(myPc, 0.01, 0.01, 0.01);
//    cout << "原始点云-栅格过滤 结束" << endl;
//
//    GF_Cloud = gaussianKernelFilter(myPc, 4, 3);
//    cout << "原始点云-高斯滤波 结束" << endl;
//
//    VGF_GF_Cloud = gaussianKernelFilter(VGF_Cloud, 4, 3);
//    cout << "原始点云-栅格过滤-高斯滤波 结束" << endl;
//
//    savePCDFile( VGF_Cloud,    "ArchVGF_0.01_0.01_0.01" );
//    savePCDFile( GF_Cloud,     "ArchGF_4_3" );
//    savePCDFile( VGF_GF_Cloud, "ArchVGF_GF_4_3");
    return 0;
    
//    myPc = bCloud;
//
//    bound = getBoundary(myPc, 20, M_PI/2);
//    boundary= extractBoundary(myPc,bound);
//    pcl::io::savePCDFile("ArchBoundariesText2.pcd", *boundary);
    
//    vector<ExtractInfo> infos = extractPlanes( myPc, 0.3, 5, 0.002 );
//    saveExtractPlanes( infos, "PlaneGroup2", "txt" );
   
    return 0;
}
