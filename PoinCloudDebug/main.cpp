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
//#include <glad/glad.h>
//#include <glfw3.h>
//#include <GL/glew.h>

using namespace std;

double xMax,yMax,zMax;
double xMin,yMin,zMin;
double xDis,yDis,zDis;

pcl::PointCloud<pcl::PointXYZ>::Ptr myPc(new pcl::PointCloud<pcl::PointXYZ>);
//void inits(){
//    glfwInit();
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
//    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
//    cout<<"To Create Window"<<endl;
//    window = glfwCreateWindow(800, 600, "LearnOpenGL", NULL, NULL);
//    cout<<"Window Created"<<endl;
//    if(window == NULL){
//        cout<<"Failed To Open A Window"<<endl;
//        glfwTerminate();
//        exit(0);
//    }
//    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
//        cout << "Failed to initialize GLAD" << endl;
//        exit(0);
//    }
//    glViewport(0, 0, 800, 600);
//}

//void glfwMyDraw(){
//    glClearColor(1, 0, 1, 0);
//    glPointSize(1);
//    glfwSwapBuffers(window);
//    glfwPollEvents();
//}

int main(int argc, const char * argv[]) {
//    pcl::PointCloud<pcl::Boundary> bound;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary;
    pcl::PointCloud<pcl::PointXYZ>::Ptr bCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr vCloud;
    
    pcl::io::loadPCDFile("ArchOrigin.pcd", *myPc);
    cout << "Source PCD File Loaded" << endl;
    
    myPc = voxelGridFilter(myPc, 0.01, 0.01, 0.01);
    pcl::io::savePCDFile("ArchVoxelGridFiltered.pcd", *myPc);
    cout << "Voxel Grid Filtered And Saved" << endl;
    
    bCloud = bilateralFilter(myPc, 5, 0.03);
    pcl::io::savePCDFile("ArchBilaterFiltered_5_0.03.pcd", *bCloud);
    cout << "Bilateral Filtered And Saved" << endl;
    
    bCloud = bilateralFilter(myPc, 5, 0.001);
    pcl::io::savePCDFile("ArchBilaterFiltered_5_0.001.pcd", *bCloud);
    cout << "Bilateral Filtered And Saved" << endl;
    
//    vCloud = voxelGridFilter(myPc, 0.001, 0.001, 0.001);
//    pcl::io::savePCDFile("ArchVoxelGridFiltered.pcd", *vCloud);
//    cout << "Voxel Grid Filtered And Saved" << endl;
    
//    vCloud = voxelGridFilter(bCloud, 0.001, 0.001, 0.001);
//    pcl::io::savePCDFile("ArchBilaterFilteredByVolexGrid.pcd", *vCloud);
//    cout << "Voxel Grid After Bilateral Filtered And Saved" << endl;
    
    return 0;
    
//    myPc = bCloud;
//
//    bound = getBoundary(myPc, 20, M_PI/2);
//    boundary= extractBoundary(myPc,bound);
//    pcl::io::savePCDFile("ArchBoundariesText2.pcd", *boundary);
//
//    bound = getBoundary(myPc, 20, M_PI/3);
//    boundary= extractBoundary(myPc,bound);
//    pcl::io::savePCDFile("ArchBoundariesText3.pcd", *boundary);
//
//    bound = getBoundary(myPc, 20, M_PI/4);
//    boundary= extractBoundary(myPc,bound);
//    pcl::io::savePCDFile("ArchBoundariesText4.pcd", *boundary);
    
//    vector<ExtractInfo> infos = extractPlanes( myPc, 0.3, 5, 0.002 );
//    saveExtractPlanes( infos, "PlaneGroup2", "txt" );
    
//    inits();
//    while(!glfwWindowShouldClose(window)) {
//        glfwMyDraw();
//    }
//    glfwTerminate();
   
    return 0;
}
