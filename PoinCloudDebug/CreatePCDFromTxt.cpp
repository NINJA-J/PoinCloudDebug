//
//  CreatePCDFromTxt.cpp
//  PoinCloudDebug
//
//  Created by Jonathan闫 on 2017/11/10.
//  Copyright © 2017年 Jonathan闫. All rights reserved.
//

#include "CreatePCDFromTxt.hpp"

using namespace std;

void buildPCD(string fName){
    ifstream fin(fName);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDWriter writer;
    
    double x,y,z;
    int points;
    
    if(!fin.is_open()){
        cout<<"Fail to open source file "<<fName<<endl;
        return;
    } else {
        cout<<"Open Source File "<<fName<<" Successfully"<<endl;
    }
    while(fin>>x>>y>>z){
        pc->points.push_back(pcl::PointXYZ(x,y,z));
        points++;
    }
    cout<<points<<" Points Found"<<endl;
    pc->width = pc->points.size();
    pc->height = 1;
    cout<<"Width of PCL is "<<pc->width<<endl;
    writer.write<pcl::PointXYZ>("Original PCL.pcd", *pc,false);
}
