/* ===========
//  * Author: Shepherd Qirong
//  * Date: 2020-05-14 22:42:43
//  * Github: https://github.com/ShepherdQR
//  * LastEditors: Shepherd Qirong
//  * LastEditTime: 2020-05-25 23:53:28
//  * Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
*/


/*  
    exercise 1: 
    //ori: 0.00399833; scan+perlin666: 0.00399843

    exercise 2: 
    ori vs scan+vox0.01
    Target ori point number is: 60269
    Target voxgrid point number is: 4647
    0.00470405

    ori vs scan+vox0.001 to 0.007
    Template ori point number is: 32497
    Target ori point number is: 60269
    Target voxgrid point number is: 6876
    0.00399833

*/


#include <algorithm>// sort
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>


#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>//minmax
#include <pcl/common/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/voxel_grid.h>

#include "FastNoise.h"//a well-known header on GitHub, The developer's email is jorzixdan.me2@gzixmail.com

//using namespace cv;
using namespace std;


string dataFolder = "/home/jellyfish/datasets/bunny/reconstruction/";
string fileName[2]= {"bunny32497.pcd",  "bunnyRe60269.pcd"  };

bool voxSwitch(true);
    float voxSize = 0.007;

bool addNoise(false);

bool savePCDS(true);

int main( int argc, char** argv ){

    
    pcl::visualization::PCLVisualizer viewer1("Viewer");
    viewer1.setBackgroundColor(255, 255, 255);// 255 *3 = white

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTem(new pcl::PointCloud<pcl::PointXYZ>), cloudTar(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::io::loadPCDFile(dataFolder + fileName[0], *cloudTem);
    cout << "Template ori point number is: " << cloudTem->points.size() << endl;//
    viewer1.addPointCloud( cloudTem, "cloud_1");
    viewer1.setPointCloudRenderingProperties( pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_1");
    viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,0, "cloud_1");


    pcl::io::loadPCDFile(dataFolder + fileName[1], *cloudTar);
    cout << "Target ori point number is: " << cloudTar->points.size() << endl;//

    if(addNoise){
        FastNoise a;
        a.SetSeed(666);
        //cout << " ==" << a.GetPerlin(2, 1, 3) << endl;
        for(size_t j=0; j<  cloudTar->points.size() ; ++j){
            float curInt = a.GetPerlin(cloudTar->points[j].x, cloudTar->points[j].x, cloudTar->points[j].z);
            cloudTar->points[j].x += curInt;
            cloudTar->points[j].y += curInt;
            cloudTar->points[j].z += curInt;
        }
    }

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarVox(new pcl::PointCloud<pcl::PointXYZ>);
    if(voxSwitch){
        pcl::VoxelGrid<pcl::PointXYZ> sor12;
        sor12.setInputCloud(cloudTar);
        sor12.setLeafSize (voxSize, voxSize, voxSize);
        sor12.filter(*cloudTarVox);
        cout << "Target voxgrid point number is: " << cloudTarVox->points.size() << endl;//
        cloudTar->clear();
        pcl::copyPointCloud(*cloudTarVox, *cloudTar);
        
    }

    viewer1.addPointCloud( cloudTar, "cloud_2");
    viewer1.setPointCloudRenderingProperties( pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_2");
    viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,0, "cloud_2");

    if(savePCDS){
        pcl::io::savePCDFile("xxx.pcd", *cloudTar);
    }


    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    int kIn = 10;
    vector<float> pointDistance(10);
    kdtree.setInputCloud(cloudTar);

    vector<int> outIndex(cloudTem->points.size());
    vector<float> outDistance(cloudTem->points.size(), 6);
    for(size_t j=0; j<  cloudTem->points.size() ; ++j){
        vector<int> pointIndex(kIn);
        kdtree.nearestKSearch (cloudTem->points[j] , kIn, pointIndex, pointDistance) ;

        //cout << "(" << cloudTar->points[j].x << ", " << cloudTar->points[j].y << ", " << cloudTar->points[j].z << ")" << endl;
        for (size_t i = 0; i < pointIndex.size (); ++i){
            float distance2(0), distance1(0);
            distance2 = pow((cloudTem->points[j].x -   cloudTar->points[ pointIndex[i] ].x), 2)+ pow((cloudTem->points[j].y -   cloudTar->points[ pointIndex[i] ].y), 2)+ pow((cloudTem->points[j].z -   cloudTar->points[ pointIndex[i] ].z), 2);
            distance1 = pow(distance2, 0.5);
            if(distance1 < outDistance[j] ){
                outDistance[j] = distance1;
                outIndex[j] =  pointIndex[i];
            }

            /*cout << "    "  <<
            cloudTem->points[ pointIndex[i] ].x << " " <<
            cloudTem->points[ pointIndex[i] ].y << " " <<
            cloudTem->points[ pointIndex[i] ].z << 
            " (squared distance: " << pointDistance[i] << ")" << distance1 <<   endl << "=======" << endl; */
        }  
    }

    float ave(0);
    for(size_t j=0; j<  cloudTem->points.size() ; ++j){
        ave += outDistance[j];
    }
    ave = ave /(cloudTem->points.size());
    cout << ave << endl; 



    while (!viewer1.wasStopped()){
        viewer1.spinOnce();
    }

    return 0;
}