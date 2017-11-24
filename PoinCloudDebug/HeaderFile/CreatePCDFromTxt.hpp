//
//  CreatePCDFromTxt.hpp
//  PoinCloudDebug
//
//  Created by Jonathan闫 on 2017/11/10.
//  Copyright © 2017年 Jonathan闫. All rights reserved.
//

#ifndef CreatePCDFromTxt_hpp
#define CreatePCDFromTxt_hpp

#include <stdio.h>

#include <iostream>
#include <fstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

void buildPCD(string fName);

#endif /* CreatePCDFromTxt_hpp */
