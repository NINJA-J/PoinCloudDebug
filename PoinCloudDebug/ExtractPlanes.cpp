//
//  ExtractPlanes.cpp
//  PoinCloudDebug
//
//  Created by Jonathan闫 on 2017/11/14.
//  Copyright © 2017年 Jonathan闫. All rights reserved.
//

#include "ExtractPlanes.hpp"

#include <fstream>
#include <iostream>

using namespace std;

ExtractInfo::ExtractInfo():
pIndices(new pcl::PointCloud<pcl::PointXYZ>),
pCoef(new pcl::ModelCoefficients) {}

vector<ExtractInfo> extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double remains, int maxPlanes, double dt){
    int planes = 0;
    int oriPoints = cloud->points.size();
    vector<ExtractInfo> ans;
    cout<<"Start Extracting Plane("<<cloud->points.size()<<" Points), "<<maxPlanes<<" Planes Expected With "<<(int)(remains*100)<<" % Points Left"<<endl;
    while( planes < maxPlanes && cloud->points.size() > oriPoints * remains ){
        ExtractInfo info = extractPlane( cloud, dt );
        double points = info.pIndices->points.size();
        ans.push_back( info );
        cout << "Plane Extracted With Para (Ax+By+Cz+D=0) : ";
        cout << points << "(";
        cout <<(int)(points/oriPoints*10000)/100.0;
        cout << " %) Points" << endl;
        cout << "    A = " << info.pCoef->values[0] << endl;
        cout << "    B = " << info.pCoef->values[1] << endl;
        cout << "    C = " << info.pCoef->values[2] << endl;
        cout << "    D = " << info.pCoef->values[3] << endl;
        planes ++;
    }
    cout<<planes<<" Planes Extracted From The Origin Cloud, ";
    cout<<(int)((double)cloud->points.size()/oriPoints*100)<<"% Points Remained"<<endl;
    return ans;
}

ExtractInfo extractPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double dt){
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sub(new pcl::PointCloud<pcl::PointXYZ>);
    ExtractInfo info;
    
    seg.setOptimizeCoefficients( true ); //可选择配置，设置模型系数需要优化
    //必须配置，设置分割的模型类型、所用的随机参数估计方法、距离阈值、输入点云
    seg.setModelType( pcl::SACMODEL_PLANE );
    seg.setMethodType ( pcl::SAC_RANSAC );
    seg.setDistanceThreshold ( dt );
    seg.setMaxIterations( 10000 ); 
    seg.setInputCloud( cloud->makeShared() );
    //引发分割实现，并存储分割结果到点集合inliers及存储平面模型的系数coefficients
    seg.segment( *inliers, *( info.pCoef ) );
    //创建点云提取对象
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud( cloud );
    extract.setIndices( inliers );
    extract.setNegative( false );
    extract.filter( *(info.pIndices) );
    
    extract.setNegative( true );
    extract.filter( *cloud_sub );
    cloud.swap( cloud_sub );
    
    return info;
}

void saveExtractPlanes( vector<ExtractInfo> &infos, string groupName, string ending ){
    pcl::PCDWriter writer;
    ofstream fout(groupName+" infos.txt");
    string fileName;
    stringstream ss;
    for(int i=0;i<infos.size();i++){
        ss.clear();
        ss << groupName << '_' << i << "."<<ending;
        ss >> fileName;
        writer.write<pcl::PointXYZ>( fileName, *infos[i].pIndices, false );
        for(int j = 0; j < 4; j++)
            fout << infos[j].pCoef->values[j] << " ";
        fout << endl;
        cout << fileName << " Saved" << endl;
    }
}
