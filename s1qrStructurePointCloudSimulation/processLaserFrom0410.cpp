/* ===========
//  * Author: Shepherd Qirong
//  * Date: 2020-04-10 17:36:56
//  * Github: https://github.com/ShepherdQR
//  * LastEditors: Shepherd Qirong
//  * LastEditTime: 2020-04-24 03:44:05
//  * Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
*/

/*//  2020-04-10 17:41:54
    Today I just wanna easily load and relocate the pcds, then I need to practice the opencv 
*/

/*//  2020-04-12 15:34:30
    I drop the idea and turn to do vox.
    The dropped idea is to make the pcd in the camera view, TO RECONSTRUCTE each point like 60 in camera. In this way the latties are used to label the point number, if we get x=0, it's easy to count the point order, and write in pcd. Generally we also need to segmente the point range, to label the number.
    Then I thought why not just using the label to vox the point.
*/

/*//  2020-04-13 02:24:41
    I finished the vox codes.
*/

/*//  2020-04-24 03:42:59
    The vox codes modified a little to supply null cloud, and I clap more clouds.
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
int usingCameraList[]={3,2,1,0};
string cameraList[4]={"00SL", "01TL", "02TR", "03SR"};
float cameraLocation[4][6]={{0.0, -M_PI_2, 0.0, -2.9, 0.0, 1.03},
                            {0.0, 0.0, 0.0, -0.6, 0.0, 3.1},
                            {0.0, 0.0, 0.0, 0.6, 0.0, 3.1},
                            {0.0, M_PI_2, 0.0, 2.9, 0.0, 1.03}};
float cameraColor[4][3]={{1,0,0}, {0,1,0}, {0,0,1}, {1,0,1}};

//int direction[4] = {};
float speed = 0.01;//  m/s
int slides = 800;// slides

bool savePcd = true;
//bool savePcd = false;

// camera
float halfAngle = 27.8; // degree
int cameraPointsNumber = (1920/4);
float AngleResolution = 2*halfAngle/cameraPointsNumber;
float maxRadius = 2.125; // metric



string currentScanPcd(const int cameraNum, const int i){
    string scanFileOut = "Scan"+ cameraList[cameraNum]+ to_string(i)+"00000.pcd";
    return  scanFileOut;
}

//void BoundaryAndResolution(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, float pMax, float pMin, float rMax, float rMin);
//float pMax, pMin, rMax, rMin; BoundaryAndResolution(cloudCur, pMax, pMin, rMax, rMin);

void transformAngleAxis( pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, float matrixIn[]);//, int matrixSize = 6


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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAllOritrans(new pcl::PointCloud<pcl::PointXYZ>),cloudAllVoxtrans(new pcl::PointCloud<pcl::PointXYZ>);

    // for(size_t ic = 0; ic< 4; ++ic)
    //     for(size_t jc = 0; jc< 3; ++jc){
    //         cameraLocation[ic][jc] *= -1.0;
    //     }
            
    timeTemp = time.toc();
    time.tic();
    timeTotal += timeTemp;
    cout << "Initialization time: " << timeTemp  << "ms." << endl;
    outfile << "Initialization time: " << timeTemp << "ms."  << endl;


    // 2  pcd load and combination ... 
    
    int cameraNumber = sizeof(usingCameraList)/sizeof(usingCameraList[0]);  //number of cameras

    for(size_t j=0; j<cameraNumber; ++j){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAll01(new pcl::PointCloud<pcl::PointXYZ>), cloudAll01trans(new pcl::PointCloud<pcl::PointXYZ>);
        vector<vector<float> > edgeResolution(5, vector<float> (slides, 0.0));// pNum, pMax, pMin, rMax, rMin;

        int isNullCloud=0;
        for(size_t i=0; i<slides ; ++i){

            // 2.1  load the raw pcd...
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCur(new pcl::PointCloud<pcl::PointXYZ>);
            string loadPcdFIle = FileFolderName + "/Scan" + cameraList[usingCameraList[j]] + "/" + currentScanPcd(usingCameraList[j], i);
            if(pcl::io::loadPCDFile(loadPcdFIle, *cloudCur)==-1){
                cloudCur->width=1;
                cloudCur->height=1;
                cloudCur->points.resize(1);
                cloudCur->points[0].x=0;
                cloudCur->points[0].y=0;
                cloudCur->points[0].z=0.35;
                isNullCloud =1;
            }
            
            


            float matrixCurIn[6]= { 0.0, 0.0, 0.0, 0.0, -1*speed*i, 0.0 };//speed * (slides-1-i)
            transformAngleAxis(cloudCur, cloudCur, matrixCurIn);

            // Eigen::Affine3f transformY = Eigen::Affine3f::Identity();
            // transformY.translation() << 0.0, speed * (slides-1-i), 0.0;
            // pcl::transformPointCloud (*cloudCur, *cloudCur, transformY);


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
            if(isNullCloud ==1){
                //edgeResolution[0][i]= 0;
                edgeResolution[4][i]=100;
                isNullCloud =0;
            }

            *cloudAll01 = *cloudAll01 + *cloudCur;//disp11
        }
        if(savePcd == true) pcl::io::savePCDFile (scanCloudPath+ "/Ori-" + cameraList[usingCameraList[j]] + ".pcd", *cloudAll01);
        *cloudAllOri = *cloudAllOri + *cloudAll01;


        // 2.3 translation and display
        float matrixCurIn2[6]={0};
            for(size_t im = 0; im< 6; ++im){
                matrixCurIn2[im]= cameraLocation[usingCameraList[j]][im];
            }       
        transformAngleAxis(cloudAll01, cloudAll01trans, matrixCurIn2);
        
        if(savePcd == true) pcl::io::savePCDFile (scanCloudPath+ "/Ori-trans" + cameraList[usingCameraList[j]] + ".pcd", *cloudAll01trans);
        *cloudAllOritrans = *cloudAllOritrans + *cloudAll01trans;
   
        viewer1.addPointCloud(cloudAll01trans , cameraList[usingCameraList[j]].c_str());
        viewer1.setPointCloudRenderingProperties( pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, cameraList[usingCameraList[j]].c_str() );
        viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, cameraColor[usingCameraList[j]][0], cameraColor[usingCameraList[j]][1], cameraColor[usingCameraList[j]][2], cameraList[usingCameraList[j]].c_str());




        // 2.4 pcd vox settings
        float cameraPMax = *max_element(edgeResolution[1].begin(), edgeResolution[1].end());
        float cameraPMin = *min_element(edgeResolution[2].begin(), edgeResolution[2].end());
        float cameraRMax = *max_element(edgeResolution[3].begin(), edgeResolution[3].end());
        float cameraRMin = *min_element(edgeResolution[4].begin(), edgeResolution[4].end());

        //
        cout <<"\n The edgeResolution information:\n" "Slides, TotalNumber, Numbers, pxMax, pxMin, rxMax, rxMin" << endl;
        int totalNumber(0);
        for ( int iqq =0; iqq< slides/slides+3 ; ++iqq ){
            totalNumber +=edgeResolution[0][iqq];
            cout << iqq << ": " << totalNumber << ", ";
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVox01(new pcl::PointCloud<pcl::PointXYZ>), cloudVox01trans(new pcl::PointCloud<pcl::PointXYZ>);
        cloudVox01->width = slides;
        cloudVox01->height = cameraPointsNumber;
        cloudVox01->points.resize(cloudVox01->width * cloudVox01->height);
        cout << cloudVox01->width * cloudVox01->height << endl;

        for(size_t i = 0; i< slides; ++i){
            for(size_t ii = 0; ii< cameraPointsNumber; ++ii){
                cloudVox01->at(i,ii).x = xList[ii];
                cloudVox01->at(i,ii).y = -1*speed*i;//speed * (slides-1-i)
                cloudVox01->at(i,ii).z = -0.35;
            }
        }//cout << cloudVox01->at(3, 6).y << endl;


        // 2.5.2  make vox pcd---02
        int LeftOrder = 0;
        for(size_t i = 0; i< slides; ++i){
            int RightOrder = LeftOrder + (int)(edgeResolution[0][i]);
            
            for(int k=LeftOrder; k <RightOrder; ++k){
                
                int OrderOO = ceil(( cameraPMax -cloudAll01->points[k].x - intStep/2)/intStep );
                //int OrderOO = (int)(floor((  cloudAll01->points[k].x -cameraPMin  +intStep/2)/intStep ));

                if(OrderOO >= cameraPointsNumber-1)
                OrderOO = cameraPointsNumber-1;// there is something wrong with 00sl camera data, while oddly others are good. I spent about 3 hours to figure it out, then I gave it up and just mute the number brustly. The amount of the overload number is not that huge.
                cloudVox01->at(i,OrderOO).z = cloudAll01->points[k].z;

            }
            LeftOrder = RightOrder;
        }
        if(savePcd == true) pcl::io::savePCDFile (scanCloudPath+ "/Vox-" + cameraList[usingCameraList[j]] + ".pcd", *cloudVox01);


        transformAngleAxis(cloudVox01, cloudVox01trans, matrixCurIn2);
        if(savePcd == true) pcl::io::savePCDFile (scanCloudPath+ "/Vox-trans-" + cameraList[usingCameraList[j]] + ".pcd", *cloudVox01trans);
        *cloudAllVoxtrans = *cloudAllVoxtrans + *cloudVox01trans;

        viewer2.addPointCloud(cloudVox01trans , cameraList[usingCameraList[j]].c_str());
        viewer2.setPointCloudRenderingProperties( pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, cameraList[usingCameraList[j]].c_str() );
        viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,cameraColor[usingCameraList[j]][0], cameraColor[usingCameraList[j]][1], cameraColor[usingCameraList[j]][2], cameraList[usingCameraList[j]].c_str());
    }


    if(savePcd == true) {
        pcl::io::savePCDFile (scanCloudPath+ "/Ori-All-Trans.pcd", *cloudAllOritrans);
        pcl::io::savePCDFile (scanCloudPath+ "/Vox-All-Trans.pcd", *cloudAllVoxtrans);
    }



viewer1.addCoordinateSystem();  
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





void transformAngleAxis( pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, float matrixIn[]){
    
    float xRot(matrixIn[0]), yRot(matrixIn[1]), zRot(matrixIn[2]), xTrans(matrixIn[3]), yTrans(matrixIn[4]), zTrans(matrixIn[5]);
    
    Eigen::Affine3f rotateX = Eigen::Affine3f::Identity();
    rotateX.rotate (Eigen::AngleAxisf (xRot, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud (*cloudIn, *cloudOut, rotateX);

    Eigen::Affine3f rotateY = Eigen::Affine3f::Identity();
    rotateY.rotate (Eigen::AngleAxisf (yRot, Eigen::Vector3f::UnitY()));
    pcl::transformPointCloud (*cloudOut, *cloudOut, rotateY);
    
    Eigen::Affine3f rotateZ = Eigen::Affine3f::Identity();
    rotateZ.rotate (Eigen::AngleAxisf (zRot, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*cloudOut, *cloudOut, rotateZ);
   
    Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
    transform1.translation() << xTrans, yTrans, zTrans;
    pcl::transformPointCloud (*cloudOut, *cloudOut, transform1);
}


            
            
            

//viewer->addCube(center,rotation,100,100,100);
       




            //disp11
            // if(i ==0){
            //     viewer1.addPointCloud(cloudAll01 , cameraList[usingCameraList[j]].c_str());
            //     viewer1.setPointCloudRenderingProperties( pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, cameraList[usingCameraList[j]].c_str() );
            //     viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0, cameraList[usingCameraList[j]].c_str());
            // }
            // else{
            //     viewer1.updatePointCloud(cloudAll01 , cameraList[usingCameraList[j]].c_str());
            //     //cout << cloudAll01->size() << endl;
            // }  





/*
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
*/









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
