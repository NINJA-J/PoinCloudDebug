//
//  Filters.cpp
//  PoinCloudDebug
//
//  Created by Jonathan闫 on 2017/11/21.
//  Copyright © 2017年 Jonathan闫. All rights reserved.
//

#include "Filters.hpp"
#include <pcl/common/impl/io.hpp>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr bilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double s,double r){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::BilateralFilter<pcl::PointXYZI> bf;
    
    pcl::copyPointCloud(*cloud, *cloud2);
    cout << "Converted To PointXYZI" << endl;
    
    bf.setInputCloud(cloud2);
    bf.setSearchMethod(tree);
    bf.setHalfSize(s);
    bf.setStdDev(r);
    cout << "Set Params Done" << endl;
    bf.filter(*cloudFiltered);
    cout << "Filtered" << endl;
    
    pcl::copyPointCloud(*cloudFiltered, *cloud3);
    cout << "Converted To PointXYZ" << endl;
    
    return cloud3;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double lx, double ly, double lz){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    
    vg.setInputCloud(cloud);
    vg.setLeafSize(lx, ly, lz);
    vg.filter(*cloudFiltered);
    
    return cloudFiltered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr gaussianKernelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, double sigma, double threshold){
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::filters::GaussianKernel<pcl::PointXYZ,pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
    pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ,pcl::PointXYZ>> convolution;
    
    kernel->setSigma( sigma );
    kernel->setThresholdRelativeToSigma( threshold );
    cout << "Kernel made" << endl;
    
    kdtree->setInputCloud(input);
    cout << "KdTree made" << endl;
    
    convolution.setKernel(*kernel);
    convolution.setInputCloud(input);
    convolution.setSearchMethod(kdtree);
    convolution.setRadiusSearch(0.01);
    cout << "Convolution Start" << endl;
    convolution.convolve(*output);
    cout << "Convoluted" << endl;
    
    return output;
}
