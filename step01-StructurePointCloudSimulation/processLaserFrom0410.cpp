/* ===========
//  * Author: Shepherd Qirong
//  * Date: 2020-04-10 17:36:56
//  * Github: https://github.com/ShepherdQR
//  * LastEditors: Shepherd Qirong
//  * LastEditTime: 2020-04-12 00:12:43
//  * Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
*/

//  2020-04-10 17:41:54
/*
    Today I just wanna easily load and relocate the pcds, then I need to practice the opencv 

*/

#include <pcl/console/time.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>//
#include <math.h>
#include <string>
#include <time.h>
#include <vector>
using namespace std;
// --------------------

string fileName = "/home/jellyfish/cpps/FinalPaperSourceCodes/AutoScanProject-20200324";
string scanFiles = fileName +"/scanFiles/";
string mode = "Linear";
string cameraList[4]={"00SL", "01TL", "02TR", "03SR"};
//int direction[4] = {};
float speed = 0.1;//  m/s
int slides = 30;// slides

// camera
float halfAngle = 27.8; // degree
int cameraPointsNumber = 60;
float AngleResolution = 2*halfAngle/cameraPointsNumber;
float maxRadius = 2.125; // metric



string currentScanPcd(const int cameraNum, const int i){
    string scanFileOut = "Scan"+ cameraList[cameraNum]+ to_string(i)+"00000.pcd";
    return  scanFileOut;
}

void BoundaryAndResolution(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, float pMax, float pMin, float rMax, float rMin);


int main(int argc, char **argv){
    pcl::console::TicToc time;
	time.tic();
    int timeTotal(0), timeTemp(0);

    // 1.1 --------- log file
    
    time_t timeLocal99 = std::time(NULL);
    char timeCurrent99[64];
    strftime(timeCurrent99, sizeof(timeCurrent99), "%Y-%m-%d%H:%M:%S",localtime(&timeLocal99));
	string timeCurrent100 = timeCurrent99;
    string fileName99 = "../logs/logs"+ timeCurrent100;
    ofstream outfile( fileName99);//c++11 write like this!.c_str
    outfile << timeCurrent99 << endl;

    string scanCloudPath = "../ScanClouds/"+timeCurrent100;
    boost::filesystem::create_directories(scanCloudPath);
    


    //  1.2  ---------- load initialization
    ifstream information(fileName+"/commonSettings/Settings.log");
    string file_temp;
    vector <string> file_inf;
    while(getline(information, file_temp)){
        if( file_temp.find("/")==-1 ||  file_temp.find("/")>=2   ){
            //cout<< file_temp;
            file_inf.push_back(file_temp);
        }
    }//cout << endl;
    string FileFolderName = scanFiles + file_inf[0];

    pcl::visualization::PCLVisualizer viewer1("Camera Viewer");
    viewer1.setBackgroundColor(255, 255, 255);// 255 *3 = white

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAllOri(new pcl::PointCloud<pcl::PointXYZ>),cloudAllVox(new pcl::PointCloud<pcl::PointXYZ>);

    timeTemp = time.toc();
    time.tic();
    timeTotal += timeTemp;
    cout << "Initialization time: " << timeTemp  << "ms." << endl;
    outfile << "Initialization time: " << timeTemp << "ms."  << endl;


    // 2.1  pcd load and combination ... 
    int usingCameraList[]={3};
    int cameraNumber = sizeof(usingCameraList)/sizeof(usingCameraList[0]);  //number of cameras

    for(size_t j=0; j<cameraNumber; ++j){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAll01(new pcl::PointCloud<pcl::PointXYZ>);
        vector<vector<float> > edgeResolution(5, vector<float> (cameraNumber, 0));// pNum, pMax, pMin, rMax, rMin;
        for(size_t i=0; i<slides ; ++i){

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCur(new pcl::PointCloud<pcl::PointXYZ>);

            string loadPcdFIle = FileFolderName + "/Scan" + cameraList[usingCameraList[j]] + "/" + currentScanPcd(usingCameraList[j], i);
            //cout << loadPcdFIle << endl;
            pcl::io::loadPCDFile(loadPcdFIle, *cloudCur);

            Eigen::Affine3f transformY = Eigen::Affine3f::Identity();
            transformY.translation() << 0.0, speed * (slides-1-i), 0.0;
            pcl::transformPointCloud (*cloudCur, *cloudCur, transformY);

            float pMax, pMin, rMax, rMin;
            BoundaryAndResolution(cloudCur, pMax, pMin, rMax, rMin);

            // edgeResolution[0].push_back((float)(cloudCur->size()));
            // edgeResolution[1].push_back(pMax);
            // edgeResolution[2].push_back(pMin);
            // edgeResolution[3].push_back(rMax);
            // edgeResolution[4].push_back(rMin);

            edgeResolution[0][j]=(float)(cloudCur->size());
            edgeResolution[1][j]=pMax;
            edgeResolution[2][j]=pMin;
            edgeResolution[3][j]=rMax;
            edgeResolution[4][j]=rMin;

            *cloudAll01 = *cloudAll01 + *cloudCur;
            if(i ==0){
                viewer1.addPointCloud(cloudAll01 , cameraList[usingCameraList[j]].c_str());
                viewer1.setPointCloudRenderingProperties( pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, cameraList[usingCameraList[j]].c_str() );
                viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0, cameraList[usingCameraList[j]].c_str());
            }
            else{
                viewer1.updatePointCloud(cloudAll01 , cameraList[usingCameraList[j]].c_str());
                //cout << cloudAll01->size() << endl;
            }  
        }
        pcl::io::savePCDFile (scanCloudPath+ "/" + cameraList[usingCameraList[j]] + "Ori.pcd", *cloudAll01);
        *cloudAllOri = *cloudAllOri + *cloudAll01;

        float cameraPMax = *max_element(edgeResolution[1].begin(), edgeResolution[1].end());
        float cameraPMin = *min_element(edgeResolution[2].begin(), edgeResolution[2].end());
        float cameraRMax = *max_element(edgeResolution[3].begin(), edgeResolution[3].end());
        float cameraRMin = *min_element(edgeResolution[4].begin(), edgeResolution[4].end());

        for ( int iqq =0; iqq< 30; ++iqq ){
        for(int iq =0; iq< 5; ++iq){
            cout << edgeResolution[iq][iqq]  << ", ";
        }
        cout << endl;
        
        }

        cout << cameraPMax << endl;

        cout << cameraPMin << endl;

        


        float intStep = (cameraPMax-cameraPMin)/(cameraPointsNumber -1);
        cout << intStep << endl;

        // vector<float> xList;//(slides)
        // for(size_t i = 0; i< slides; ++i){
        //     xList.push_back(cameraPMax - i * intStep );
        // }

        

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVox01(new pcl::PointCloud<pcl::PointXYZ>);
        cloudVox01->width = cameraPointsNumber;
        cloudVox01->height = slides;
        cloudVox01->points.resize(cloudVox01->width * cloudVox01->height);

        int LeftOrder = 0;
        
        for(size_t i = 0; i< slides; ++i){
            // every line...
            int labelLattie[2][cameraPointsNumber]={0};

            int RightOrder = LeftOrder + (int)(edgeResolution[0][i]);
            for(size_t k=LeftOrder; k <RightOrder; ++k){
                int OrderOO = ceil((cloudAllOri->points[k].x - intStep/2)/intStep );
                if(OrderOO < 0)
                    OrderOO =0;
                
                //cout << OrderOO << endl;
                labelLattie[0][OrderOO] = 1;
                labelLattie[1][OrderOO] = k;
            }

            //cout << "hi3" << endl; ////

            // for(size_t ii = 0; ii< cloudVox01->width; ++ii){

            //     if(labelLattie[0][ii] == 0){
            //         cloudVox01->at(ii,i).x = 0.0;
            //         cloudVox01->at(ii,i).y = 0.0;
            //         cloudVox01->at(ii,i).z = 0.0;
            //     }
            //     else{
            //         cloudVox01->at(ii,i).x = cloudAllOri->points[labelLattie[1][ii]].x;
            //         cloudVox01->at(ii,i).y = cloudAllOri->points[labelLattie[1][ii]].y;
            //         cloudVox01->at(ii,i).z = cloudAllOri->points[labelLattie[1][ii]].z;
            //     }
            // }
            //cout << "hi4" << endl;
            LeftOrder = RightOrder;
        }
        
    }
    
    

    

    //cout << currentScanPcd(3, 1) << endl;

    while (!viewer1.wasStopped()  ){
		viewer1.spinOnce();
	}
    timeTemp = time.toc();
    timeTotal += timeTemp;
    cout << "Total time: " << timeTotal << "ms." << endl;
    outfile << "Total time: " << timeTotal << "ms." << endl;
    outfile.close();
    return 0;
}


void BoundaryAndResolution(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, float pMax, float pMin, float rMax, float rMin){
    pMax = cloudIn->points[0].x;
    pMin = cloudIn->points[cloudIn->size()-1].x;
    rMax = 0.0;
    rMin = 100.0;

    for(size_t i=0; i < cloudIn->size()-1; ++i){
        float resolutionI = cloudIn->points[i].x - cloudIn->points[i+1].x;
        rMax =  resolutionI > rMax ? resolutionI : rMax;
        rMin =  resolutionI < rMin ? resolutionI : rMin;
    }
}











