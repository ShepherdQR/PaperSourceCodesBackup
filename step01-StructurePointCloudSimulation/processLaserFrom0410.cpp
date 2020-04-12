/* ===========
//  * Author: Shepherd Qirong
//  * Date: 2020-04-10 17:36:56
//  * Github: https://github.com/ShepherdQR
//  * LastEditors: Shepherd Qirong
//  * LastEditTime: 2020-04-13 02:25:25
//  * Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
*/

//  2020-04-10 17:41:54
/*
    Today I just wanna easily load and relocate the pcds, then I need to practice the opencv 
*/
//  2020-04-12 15:34:30
/*
    I drop the idea and turn to do vox.
    The dropped idea is to make the pcd in the camera view, TO RECONSTRUCTE each point like 60 in camera. In this way the latties are used to label the point number, if we get x=0, it's easy to count the point order, and write in pcd. Generally we also need to segmente the point range, to label the number.
    Then I thought why not just using the label to vox the point.
*/

//  2020-04-13 02:24:41
/*
    I finished the vox codes.
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

bool savePcd = true;
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
//float pMax, pMin, rMax, rMin; BoundaryAndResolution(cloudCur, pMax, pMin, rMax, rMin);


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

    pcl::visualization::PCLVisualizer viewer1("Ori"), viewer2("Vox");
    viewer1.setBackgroundColor(255, 255, 255);// 255 *3 = white
    viewer2.setBackgroundColor(255, 255, 255);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAllOri(new pcl::PointCloud<pcl::PointXYZ>),cloudAllVox(new pcl::PointCloud<pcl::PointXYZ>);

    timeTemp = time.toc();
    time.tic();
    timeTotal += timeTemp;
    cout << "Initialization time: " << timeTemp  << "ms." << endl;
    outfile << "Initialization time: " << timeTemp << "ms."  << endl;


    // 2  pcd load and combination ... 
    int usingCameraList[]={3};
    int cameraNumber = sizeof(usingCameraList)/sizeof(usingCameraList[0]);  //number of cameras

    for(size_t j=0; j<cameraNumber; ++j){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAll01(new pcl::PointCloud<pcl::PointXYZ>);
        vector<vector<float> > edgeResolution(5, vector<float> (slides, 0.0));// pNum, pMax, pMin, rMax, rMin;
        for(size_t i=0; i<slides ; ++i){

            // 2.1  load the raw pcd...
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCur(new pcl::PointCloud<pcl::PointXYZ>);
            string loadPcdFIle = FileFolderName + "/Scan" + cameraList[usingCameraList[j]] + "/" + currentScanPcd(usingCameraList[j], i);
            pcl::io::loadPCDFile(loadPcdFIle, *cloudCur);

            Eigen::Affine3f transformY = Eigen::Affine3f::Identity();
            transformY.translation() << 0.0, speed * (slides-1-i), 0.0;
            pcl::transformPointCloud (*cloudCur, *cloudCur, transformY);


            // 2.2  number, boundary, resolution of the raw pcd...
            float pMax = cloudCur->points[0].x;
            float pMin = cloudCur->points[cloudCur->size()-1].x;
            float rMax = 0.0, rMin = 100.0;
            for(size_t ir=0; ir < cloudCur->size()-1; ++ir){
                float resolutionI = cloudCur->points[ir].x - cloudCur->points[ir+1].x;
                resolutionI = (resolutionI > 0.0000001) ? resolutionI : (-resolutionI);
                rMax =  (resolutionI > rMax) ? resolutionI : rMax;
                rMin =  (resolutionI < rMin) ? resolutionI : rMin;
                //the "()" cost me about 2 hours to fix!
            }

            edgeResolution[0][i]=(float)(cloudCur->size());
            edgeResolution[1][i]=pMax;
            edgeResolution[2][i]=pMin;
            edgeResolution[3][i]=rMax;
            edgeResolution[4][i]=rMin;

            
            // 2.3  combination1 and display of the raw pcd...
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
        if(savePcd == true) pcl::io::savePCDFile (scanCloudPath+ "/Ori-" + cameraList[usingCameraList[j]] + ".pcd", *cloudAll01);
        *cloudAllOri = *cloudAllOri + *cloudAll01;
        




        // 2.4 pcd vox settings
        float cameraPMax = *max_element(edgeResolution[1].begin(), edgeResolution[1].end());
        float cameraPMin = *min_element(edgeResolution[2].begin(), edgeResolution[2].end());
        float cameraRMax = *max_element(edgeResolution[3].begin(), edgeResolution[3].end());
        float cameraRMin = *min_element(edgeResolution[4].begin(), edgeResolution[4].end());

        cout <<"\n The edgeResolution information:\n" "Numbers, pxMax, pxMin, rxMax, rxMin" << endl;
        for ( int iqq =0; iqq< slides; ++iqq ){
        for(int iq =0; iq< 5; ++iq){
            cout << edgeResolution[iq][iqq]  << ", ";
        }
        cout << endl;
        }
        cout << "<<, " << cameraPMax << ", " << cameraPMin << ", " << cameraRMax << ", " << cameraRMin << " >>" << endl;

        float intStep = (cameraPMax-cameraPMin)/(cameraPointsNumber -1);
        vector<float> xList(cameraPointsNumber, 0.0);//
        for(size_t i = 0; i< cameraPointsNumber; ++i){
            xList[i] = cameraPMax - i * intStep;
        }
        cout <<"Lattie length = " << intStep << ", X Range is: " << xList[0] << ", " << xList[cameraPointsNumber-1] << endl;

        
        // 2.5.1  make vox pcd---01
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVox01(new pcl::PointCloud<pcl::PointXYZ>);
        cloudVox01->width = slides;
        cloudVox01->height = cameraPointsNumber;
        cloudVox01->points.resize(cloudVox01->width * cloudVox01->height);

        for(size_t i = 0; i< slides; ++i){
            for(size_t ii = 0; ii< cameraPointsNumber; ++ii){
                cloudVox01->at(i,ii).x = xList[ii];
                cloudVox01->at(i,ii).y = speed * (slides-1-i);
                cloudVox01->at(i,ii).z = 0.0;
            }
        }//cout << cloudVox01->at(3, 6).y << endl;


        // 2.5.2  make vox pcd---02
        int LeftOrder = 0;
        for(size_t i = 0; i< slides; ++i){
            int RightOrder = LeftOrder + (int)(edgeResolution[0][i]);
            for(size_t k=LeftOrder; k <RightOrder; ++k){
                int OrderOO = ceil(( cameraPMax -cloudAll01->points[k].x - intStep/2)/intStep );
                cloudVox01->at(i,OrderOO).z = cloudAll01->points[k].z;
            }
            LeftOrder = RightOrder;
        }
        if(savePcd == true) pcl::io::savePCDFile (scanCloudPath+ "/Vox-" + cameraList[usingCameraList[j]] + ".pcd", *cloudVox01);

        *cloudAllVox = *cloudAllVox + *cloudVox01;
        viewer2.addPointCloud(cloudVox01 , cameraList[usingCameraList[j]].c_str());
        viewer2.setPointCloudRenderingProperties( pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, cameraList[usingCameraList[j]].c_str() );
        viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0, cameraList[usingCameraList[j]].c_str());
    }


    if(savePcd == true) {
        pcl::io::savePCDFile (scanCloudPath+ "/Ori-All.pcd", *cloudAllOri);
        pcl::io::savePCDFile (scanCloudPath+ "/Vox-All.pcd", *cloudAllVox);
    }




    while ((!viewer1.wasStopped()) & (!viewer2.wasStopped()) ){
		viewer1.spinOnce();
        viewer2.spinOnce();
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

    for(size_t ir=0; ir < cloudIn->size()-1; ++ir){
        float resolutionI = cloudIn->points[ir].x - cloudIn->points[ir+1].x;
        resolutionI = (resolutionI > 0.0000001) ? resolutionI : (-resolutionI);
        rMax =  (resolutionI > rMax) ? resolutionI : rMax;
        rMin =  (resolutionI < rMin) ? resolutionI : rMin;
        //the "()" cost me about 2 hours to fix!
    }
}










//====================   useless
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
