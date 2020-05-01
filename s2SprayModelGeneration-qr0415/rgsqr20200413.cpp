/* ===========
//  * Author: Shepherd Qirong
//  * Date: 2020-04-13 23:52:28
//  * Github: https://github.com/ShepherdQR
//  * LastEditors: Shepherd Qirong
//  * LastEditTime: 2020-05-01 17:16:24
//  * Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
*/

/*//  2020-04-13 23:53:31
    Seperate this part
*/

/*//  2020-04-15 01:50:29
    Yesterday I drop the idea to do rgs in opencv, cause the process is more easier in the preprocess of the pattern pictures.
    And for the display, I can do it using opengl or other lib, but to simply the process, I just add cubes with pcl.
*/

/*//  2020-04-15 03:52:29
    I have a better way. And I'll fix it tommorrow.
*/

/*//  2020-04-17 03:04:11
    I have finished the preprocess of the pictures, using inkscape and opencv.
    //(int)(i-indi)> 0 size_t or int in for is unsign int !!
    // the color in green is not 255!
    // in cv the order is bgr!
    Tommorrow I use opengl to render, due to the limits of pcl.
*/

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>


#include <pcl/console/time.h>   // TicToc
#include <pcl/features/normal_3d.h>


#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>



#include <boost/filesystem.hpp>//

#include <algorithm>// sort
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>

using namespace cv;
using namespace std;


void index2Color(Mat imageFill, int ii, int jj, int colorLabelP);
int colorLabelF(Vec3b intensity);



string picsFolder = "/home/jellyfish/cpps/FinalPaperSourceCodes/Pics/";
string PicFileName = "LeftBottomRGBY03";


string fileName1 = "/home/jellyfish/cpps/FinalPaperSourceCodes/s1StructurePointCloudSimulation-qr0410";
string scanFiles = fileName1 +"/ScanClouds/2020-04-2403:32:11";

string whichPcd2Load = "/Ori-trans00SL.pcd";
//"/Vox-trans-00SL.pcd"

string projectFolder = "/home/jellyfish/cpps/FinalPaperSourceCodes/s2SprayModelGeneration-qr0415/";
 

bool savePcds = false;


bool step1 = true;


float colorList[6][3]={1, 0, 0,
        0, 1, 0,    0, 0, 1,     1, 1, 0,
        0.1, 0.2, 0.3,  0.4, 0.2, 0.3};


int main( int argc, char** argv ){


    // ==  1    initialization
    // =================================
    // 1.1  time & log initialization--------
        pcl::console::TicToc time;
        time.tic();
        int timeTotal(0);

        time_t timeLocal99 = std::time(NULL);
        char timeCurrent99[64];
        strftime(timeCurrent99, sizeof(timeCurrent99), "%Y-%m-%d%H:%M:%S",localtime(&timeLocal99));
        string timeCurrent100 = timeCurrent99;
        string processedPath = projectFolder + "Processed/" + timeCurrent100;
        boost::filesystem::create_directories(processedPath);

        string fileName99 = processedPath + "/"+ timeCurrent100 + ".log";
        ofstream outfile( fileName99);//c++11 write like this!.c_str
        outfile << timeCurrent99 << endl;
    //---------------------------


    // 1.2  general initialization----------------
        ifstream information("/home/jellyfish/cpps/FinalPaperSourceCodes/AutoScanProject-20200324/commonSettings/Settings.log");
        string file_temp;
        vector <string> file_inf;
        while(getline(information, file_temp)){
            if( file_temp.find("/")==-1 ||  file_temp.find("/")>=2   ){
                //cout<< file_temp;
                file_inf.push_back(file_temp);
            }
        }//cout << endl;
        string FileFolderName = scanFiles;
        
        pcl::visualization::PCLVisualizer viewer1("Ori"), viewer3("segmentation");
        viewer1.setBackgroundColor(255, 255, 255);// 255 *3 = white
        viewer3.setBackgroundColor(255, 255, 255);// 255 *3 = white

        // pcl::visualization::PCLVisualizer  viewer2("RegionGrowing");
        // int v21(0),v22(1),v23(2),v24(3);
        // viewer2.createViewPort(0.0, 0.0, 0.5, 0.5, v23);//bottomleft=(0,0)
        // viewer2.createViewPort(0.5, 0.0, 1.0, 0.5, v24);
        // viewer2.createViewPort(0.0, 0.5, 0.5, 1.0, v21);
        // viewer2.createViewPort(0.5, 0.5, 1.0, 1.0, v22);
        // viewer2.setBackgroundColor(255, 255, 255);


        int time_visual_init=time.toc();
        timeTotal += time_visual_init; 
        cout<<"Visualization Initialization costs time: "<<time_visual_init<<" ms."<< endl;
        outfile<<"Visualization Initialization costs time: "<<time_visual_init<< " ms."<< endl;
    //---------------------
// ==============================

    if(step1){
    // ==  2    segementation
    // =================================
    // 2.1 pcd load and disp------------
        time.tic();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOri(new pcl::PointCloud<pcl::PointXYZ>);
        string loadPcdFIle = FileFolderName + whichPcd2Load;
        pcl::io::loadPCDFile(loadPcdFIle, *cloudOri);
        cout << loadPcdFIle << endl;

        viewer1.addPointCloud(cloudOri, "ori");
        viewer1.setPointCloudRenderingProperties( pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, "ori");
        viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,1, "ori");

        int time_disp=time.toc();
        timeTotal += time_disp; 
        cout<<"To display origional clouds costs time: "<<time_disp<<" ms."<< endl;
        outfile<<"To display origional clouds costs time: "<<time_disp<< " ms."<< endl;
    //---------------


    // 2.2 RegionGrowing --------------in: cloudOri
        time.tic();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz_ori_filter(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloudOri, *cloudxyz_ori_filter);

        pcl::search::Search<pcl::PointXYZ>::Ptr tree_ori_filter = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud <pcl::Normal>::Ptr normals_ori_filter (new pcl::PointCloud <pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_ori_filter;
        normal_ori_filter.setSearchMethod (tree_ori_filter);
        normal_ori_filter.setInputCloud ( cloudxyz_ori_filter );
        normal_ori_filter.setKSearch (50);
        normal_ori_filter.compute (*normals_ori_filter);
        cout<<"  Gotcha the normals of the cloud after filtering."<<endl;
        outfile<<"  Gotcha the normals of the cloud after filtering."<<endl;

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg_ori_filter;
        reg_ori_filter.setMinClusterSize (2500);//800 box
        reg_ori_filter.setMaxClusterSize (3000000);
        reg_ori_filter.setSearchMethod (tree_ori_filter);
        reg_ori_filter.setNumberOfNeighbours (50);
        reg_ori_filter.setInputCloud (cloudxyz_ori_filter);
        reg_ori_filter.setInputNormals (normals_ori_filter);
        reg_ori_filter.setSmoothnessThreshold (10.0 / 180.0 * M_PI);//senstive
        reg_ori_filter.setCurvatureThreshold (4.0);//4

        std::vector <pcl::PointIndices> clusterOri;
        reg_ori_filter.extract (clusterOri);

        int time_rgs=time.toc();
        timeTotal += time_rgs; 
        cout<<"To segmentate clouds with rgs costs time: "<<time_rgs<<" ms."<< endl;
        outfile<<"To segmentate clouds with rgs costs time: "<<time_rgs<< " ms."<< endl;
    // ---------------------------------out: clusterOri
        

    // 2.3 cluster sort --------------
        time.tic();    
        int numRows_cluster_container= clusterOri.size(), numClo_cluster_container=2;
		vector<vector<int> > vector_cluster_before(numRows_cluster_container, vector<int>());
        for (size_t i=0; i!=numRows_cluster_container; ++i)
		vector_cluster_before[i].resize(numClo_cluster_container);

        for (size_t i=0; i!=clusterOri.size();++i){
            vector_cluster_before[i][0]=clusterOri[i].indices.size();
            vector_cluster_before[i][1]=i;
        }
        
        sort( vector_cluster_before.begin(), vector_cluster_before.end());
        sort( vector_cluster_before.rbegin(), vector_cluster_before.rend());

        for (size_t i=0; i!=clusterOri.size();++i){
            cout << "The first " << clusterOri.size() << " subsets, the point number and the cluster order: " << endl;
            outfile << "The first " << clusterOri.size() << " subsets, the point number and the cluster order: " << endl;
            cout << vector_cluster_before[i][0] <<", "<<
            vector_cluster_before[i][1] << endl;
            outfile << vector_cluster_before[i][0] <<", "<<
            vector_cluster_before[i][1] << endl;
        }

        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSLB(new pcl::PointCloud<pcl::PointXYZ>), cloudSLT(new pcl::PointCloud<pcl::PointXYZ>);
        for(size_t i=0; i<4; ++i){
            std::stringstream ss_i;
	        ss_i  << i;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMain(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloudOri, clusterOri[vector_cluster_before[i][1]], *cloudMain );
            if(savePcds){
                pcl::io::savePCDFile(processedPath +"/segmentation" + ss_i.str()+".pcd",*cloudMain);
                cout << "pcd is saved in: " << processedPath +"/segmentation" << ss_i.str() << ".pcd" << endl;
                outfile << "pcd is saved in: " << processedPath +"/segmentation" << ss_i.str() << ".pcd" << endl;
            }
                
            viewer3.addPointCloud(cloudMain, "cloudv"+ ss_i.str()  );
            viewer3.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, "cloudv"+ ss_i.str());

            //srand(12*i+1);float value1 = rand()/999999999;
            //srand(12*i+2);float value2 = rand()/999999999;
            //srand(12*i+3);float value3 = rand()/999999999;
            viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colorList[i][0], colorList[i][1], colorList[i][2], "cloudv"+ ss_i.str());

            cout << "  the " << i << "th cluster's point number is: " <<cloudMain->width * cloudMain->height << endl;
            outfile << "  the " << i << "th cluster's point number is: " <<cloudMain->width * cloudMain->height << endl;

            if((i==0) | (i==1)){
                *cloudSLB = *cloudSLB + *cloudMain;
            }
            if((i==2) | (i==3)){
                *cloudSLT = *cloudSLT + *cloudMain;
            }
        }

        if(savePcds){
            pcl::io::savePCDFile(processedPath +"/segmentation-SLB.pcd",*cloudSLB);
            pcl::io::savePCDFile(processedPath +"/segmentation-SLT.pcd",*cloudSLT);
            cout << "pcd is saved in: " << processedPath +"/segmentation-SLB and SLT" << endl;
            cout << "pcd is saved in: " << processedPath +"/segmentation-SLB and SLT" << endl;
        }

        int time_sort=time.toc(); 
        timeTotal+= time_sort;
        cout<<"  Sorting clouds costs time: "<<time_sort<<" ms."<< endl;
        outfile<<"  Sorting clouds costs time: "<<time_sort<< " ms."<< endl;
    //---------------------------
    }



    // if(step2){
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     if(step1){

    //     }
    // }







    






    
    // // 2. segmention along z direction
    // // 2.1 find z max
    // vector <float> cloudsl;
    // for(size_t i=0; i<(cloudCur->width); ++i){
    //     float zMin1 = 0;
    //     for(size_t j=0; j<(cloudCur->height); ++j){
    //         zMin1 = (cloudCur->at(i,j).z < zMin1)? (cloudCur->at(i,j).z): zMin1;
    //     }
    //     cloudsl.push_back(zMin1);
    // }
    // cout << "z max begins.." << endl;
    // for(size_t i=0; i<cloudsl.size(); ++i){
    //     cout << cloudsl[i] << endl;
    // }
    // cout << "z max ends.." << endl;

    // // 2.2 segmentions
    // vector <int>  planeSLB;// plan side left bottom
    // for(size_t i=0; i< cloudsl.size()-2; ++i){
    //     if( (cloudsl[i]-cloudsl[i+1]) * (cloudsl[i]-cloudsl[i+1])  > 0.3 * 0.3){
    //         planeSLB.push_back(i);
    //     }
    // }
    // for(size_t i=0; i<planeSLB.size(); ++i){
    //     cout << planeSLB[i] << ", " << cloudsl[ planeSLB[i]] << endl;
    // }/*
    //     5, -1.31545
    //     7, -1.63126
    //     21, -1.21162
    //     54, -1.90665
    //     64, -1.1571
    //     68, -1.67544
    //     73, -1.23865
    // */
    

    

//====================








    // string imageName( picsFolder + PicFileName +  ".png" );
    // Mat iIn= imread( imageName ); 
    // //default = IMREAD_COLORIM.   READ_GRAYSCALE
    // //CV_UNUSED(iIn);

    // namedWindow( "Display window", WINDOW_AUTOSIZE ); 
    // imshow( "Display window", iIn );  

    // int nRows = iIn.rows;
    // int nCols = iIn.cols;
    // cout << nRows << endl;


    // Mat imageFill= iIn.clone();
    // //Mat imageFillCopy= imageFill.clone();

    // for(size_t  i = 0; i < nRows; ++i){
    //     for(size_t j = 0; j < nCols; ++j){
    //         Vec3b intensity = imageFill.at<Vec3b>(i, j);
    //         // float blue1 = intensity.val[0];
    //         // float green1 = intensity.val[1];
    //         // float red1 = intensity.val[2];
    //         int colorLabel = colorLabelF( intensity);
            
    //         if( colorLabel == 4){
    //             int whichColor[2][2]={{4,nRows},{4,nRows}};
    //             int colorLabelP=4;

    //             int s11=0, s12=0;
    //             for(size_t indi = 1; indi < nCols/3; ++indi){
    //                 if(s11 || s12)  break;
    //                 if(((int)(i-indi)> 0) & (s11==0)){
    //                     Vec3b intensityP1 = imageFill.at<Vec3b>((int)(i-indi), j);
    //                     int colorLabelP1 = colorLabelF( intensityP1 );
    //                     if( colorLabelP1 != 4 ){
    //                         whichColor[0][0]= colorLabelP1;
    //                         whichColor[0][1]= -1 * indi;
    //                         s11 =1;
    //                     } 
    //                 }
    //                 if((i+indi) < nRows){
    //                     Vec3b intensityP2 = imageFill.at<Vec3b>((i+indi), j);
    //                     int colorLabelP2 = colorLabelF( intensityP2 );
    //                     if(  colorLabelP2 != 4 & (s12==0)){
    //                         whichColor[1][0]= colorLabelP2;
    //                         whichColor[1][1]= indi;
    //                         s12 = 1;
    //                     } 
    //                 }  
    //             }
    //             if((whichColor[0][1] * whichColor[0][1]) < (whichColor[1][1] * whichColor[1][1]))
    //                 colorLabelP = whichColor[0][0];
    //             else colorLabelP = whichColor[1][0];

    //             index2Color(imageFill, i, j, colorLabelP);
    //             // imageFill.at<Vec3b>(i, j).val[0]=255;
    //             // imageFill.at<Vec3b>(i, j).val[1]=255;
    //             // imageFill.at<Vec3b>(i, j).val[2]=255;
    //         }
    //     }
    // }
    // imshow("imageFill",  imageFill);
    // //imwrite(picsFolder + PicFileName +  "Narrow.png", imageFill);


    // //Mat iOut;
    // //int rWidth = cloudCur->width , rHeight = cloudCur->height;
    // //cv::resize(imageFill, iOut, cv::Size( rWidth, round(rHeight*0.5)), INTER_NEAREST );//INTER_LINEAR  INTER_AREA
    // // imshow("imageOut", iOut);










//=============================

















    // float lattieX = 0.0301336, lattieY = 0.1;



    // float ky= nRows /(planeSLB[3] - planeSLB[2]   ), kx =  nCols /(cloudCur->width );// based on the search we use 2 and 3 clusters
    // float pUVRange[4]={cloudCur->at((cloudCur->width)-1, planeSLB[3]+1 ).x,cloudCur->at((cloudCur->width)-1, planeSLB[3]+1 ).y, cloudCur->at(0, planeSLB[2]).x, cloudCur->at(0, planeSLB[2] ).y };
    // cout << "LT and RB:" << 
    // pUVRange[0] << "; " << pUVRange[1] << "; " <<
    // pUVRange[2] << "," << pUVRange[3] << "," << endl;
    // //-0.676124; -7.9; 0.348419,-0

    // float informationUV[]={0.1, 0.1};//du,dv;
    // int nU= ceil( (pUVRange[3] - pUVRange[1])/informationUV[0]);
    // int nV= ceil( (pUVRange[2] - pUVRange[0])/informationUV[1]);
    // //cccc


    
    // //string imageName1( picsFolder + PicFileName +  "Narrow.png" );
    // //Mat imageFill1= imread( imageName1 ); 


   
    // // points starts from planeSLB[1]  to planeSLB[2], 
    // int nn(0);
    // for(size_t i=0; i<(cloudCur->width); ++i ){
    //     for(size_t jj=planeSLB[2]+1 ; jj<planeSLB[3]+1; ++jj ){
    //         ++nn;
    //         stringstream ssn;
    //         ssn<<nn; 
    //         string name="cube"+ssn.str();

    //         int atj=  kx * i,ati =  ky*jj; 
    //         //nCols -
    //         //nRows -
    //         // points origion is rightbottom, while image is lefttop

    //         double colorR=0, colorG=0, colorB=0;
    //         if(imageFill.at<Vec3b>(ati, atj).val[2]>145 ){
    //             colorR=1;
    //         }
    //         if(imageFill.at<Vec3b>(ati, atj).val[1]>145 ){
    //             colorG=1;
    //         }
    //         if(imageFill.at<Vec3b>(ati, atj).val[0]>145 ){
    //             colorB=1;
    //         }
    //         //double colorR=imageFill.at<Vec3b>(ati, atj).val[2]/255, colorG=imageFill.at<Vec3b>(ati, atj).val[1]/255,colorB=imageFill.at<Vec3b>(ati, atj).val[0]/255;

    //         // if(i==20){
    //         //     cout << ati << endl;
    //         //     cout << colorR << colorG << colorB << endl;
    //         // }


    
    //         // if(cloudCur->at(i,jj).z !=0){
    //         //     viewer2.addCube (cloudCur->at(i,jj).x-0.03, cloudCur->at(i,jj).x, cloudCur->at(i,jj).y-0.1, cloudCur->at(i,jj).y, cloudCur->at(i,jj).z, cloudCur->at(i,jj).z+0.01, colorR, colorG, colorB, name);
    //         // }

    //         viewer2.addCube (cloudCur->at(i,jj).x-0.03, cloudCur->at(i,jj).x, cloudCur->at(i,jj).y-0.1, cloudCur->at(i,jj).y, cloudCur->at(i,jj).z, cloudCur->at(i,jj).z+0.01, colorR, colorG, colorB, name);

    //     }
    // }



    
    //waitKey();
    viewer1.addCoordinateSystem();
    viewer3.addCoordinateSystem();
    while ((!viewer1.wasStopped()) & (!viewer3.wasStopped()) ){
		viewer1.spinOnce();
        
	}
    


    outfile.close();
    return 0;
}


void index2Color(Mat imageFill, int ii, int jj,  int colorLabelP){
    //Vec3b intensity = imageFill.at<Vec3b>(ii, jj);
    if(colorLabelP==0){
        imageFill.at<Vec3b>(ii, jj).val[0]=0;
        imageFill.at<Vec3b>(ii, jj).val[1]=0;
        imageFill.at<Vec3b>(ii, jj).val[2]=255;
    }else if(colorLabelP==1){
        imageFill.at<Vec3b>(ii, jj).val[0]=0;
        imageFill.at<Vec3b>(ii, jj).val[1]=255;
        imageFill.at<Vec3b>(ii, jj).val[2]=0;
    }else if(colorLabelP==2){
        imageFill.at<Vec3b>(ii, jj).val[0]=255;
        imageFill.at<Vec3b>(ii, jj).val[1]=0;
        imageFill.at<Vec3b>(ii, jj).val[2]=0;
    }else if(colorLabelP==3){
        imageFill.at<Vec3b>(ii, jj).val[0]=0;
        imageFill.at<Vec3b>(ii, jj).val[1]=255;
        imageFill.at<Vec3b>(ii, jj).val[2]=255;
    }
}



int colorLabelF(Vec3b intensity){

    int colorLabel(4);
    float blue = intensity.val[0];
    float green = intensity.val[1];
    float red = intensity.val[2];
    if((red==0) & (green==0) & (blue==255)){
        colorLabel=2;
    }else if((red==255) & (green==0) & (blue==0)){
        colorLabel=0;
    }else if((red==0) & (green==255) & (blue==0)){
        colorLabel=1;
    }else if((red==255) & (green==255) & (blue==0)){
        colorLabel=3;
    }
    return colorLabel;
}



//addCube (float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "cube", int viewport = 0);









    // // 2.4 Outlier Removal
    // time.tic();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPro1(new pcl::PointCloud<pcl::PointXYZ>);
    // sor_outlierremoval.setInputCloud (cloudOri);
    // sor_outlierremoval.setMeanK ( 20 );
    // sor_outlierremoval.setStddevMulThresh (1.0);//1sigma
    // sor_outlierremoval.filter (*cloudPro1);

    // cout << "  number of points after Outlier Removal: " <<cloudPro1->width * cloudPro1->height << endl;
    // outfile << "  number of points after Outlier Removal: " <<cloudPro1->width * cloudPro1->height << endl;

    // int time_pro1=time.toc();
    // timeTotal += time_pro1; 
    // cout<<"Outlier Removal costs time: "<<time_pro1<<" ms."<< endl;
	// outfile<<"Outlier Removal costs time: "<<time_pro1<< " ms."<< endl;




















// if( colorLabel == 4){
//                 int colorPixFlag(0), colorLabel1(4);
//                 for (size_t k=0; k< nRows/5; ++k){
//                     if(colorPixFlag ==1) break;
                    
//                     int directionRow[2]={-1, 1};
//                     for(size_t idi=0; idi<2; ++idi){
//                         int pIndix = i + directionRow[idi] * k;
//                         //cout << pIndix << endl;
//                         if((pIndix >0 ) & (pIndix < nRows ) & (colorPixFlag==0)){
//                             Vec3b pNeighbor = imageFill.at<Vec3b>(pIndix, j);
//                             colorLabel1 = colorLabelF( pNeighbor);
//                             if(colorLabel1 !=4){
//                                 colorPixFlag=1;
//                             }
//                         }
//                         if(colorPixFlag ==1) break;
//                     } 
//                     if(colorPixFlag ==1) break;
//                 }
//                 //cout << colorLabel << endl;

//                 if(colorLabel1==0){
//                     imageFill.at<Vec3b>(i, j).val[0]=0;
//                     imageFill.at<Vec3b>(i, j).val[1]=0;
//                     imageFill.at<Vec3b>(i, j).val[2]=255;
//                 }else if(colorLabel1==1){
//                     imageFill.at<Vec3b>(i, j).val[0]=0;
//                     imageFill.at<Vec3b>(i, j).val[1]=255;
//                     imageFill.at<Vec3b>(i, j).val[2]=0;
//                 }else if(colorLabel1==2){
//                     imageFill.at<Vec3b>(i, j).val[0]=255;
//                     imageFill.at<Vec3b>(i, j).val[1]=0;
//                     imageFill.at<Vec3b>(i, j).val[2]=0;
//                 }else if(colorLabel1==3){
//                     imageFill.at<Vec3b>(i, j).val[0]=255;
//                     imageFill.at<Vec3b>(i, j).val[1]=255;
//                     imageFill.at<Vec3b>(i, j).val[2]=0;
//                 }

//             }










//  for(size_t i = 0; i < nRows; ++i){
//         Vec3b intensity = iIn.at<Vec3b>(i, 0);
//         float blue = intensity.val[0];
//         float green = intensity.val[1];
//         float red = intensity.val[2];
//         if( red != 255){
//             cout << blue  << ",," << green << ",," << red <<endl;
//         }
       
//     }




// addX01
            // if((blue == 0) & (green == 0) & (red == 0)){
            //     int colorFLag[4]={0};//RGBY
            //     for(size_t k = 0; k < numberNeighbor; ++k){
            //         if((i+neighborList[k][0] >=0 ) & (i+neighborList[k][0] <=nCols ) & (j+neighborList[k][1] >=0 ) & (j+neighborList[k][1] <=nRows ) ){
            //             Vec3b pNeighbor = imageFill.at<Vec3b>(i+neighborList[k][0], j+neighborList[k][1]);
            //             float blue = pNeighbor.val[0];
            //             float green = pNeighbor.val[1];
            //             float red = pNeighbor.val[2];
            //             if(blue==255){
            //                 colorFLag[2] +=1;
            //                 continue;
            //             }if((red==255) & (green==0)){
            //                 colorFLag[0] +=1;
            //                 continue;
            //             }if((red==0) & (green==255)){
            //                 colorFLag[1] +=1;
            //                 continue;
            //             }if((red==255) & (green==255)){
            //                 colorFLag[3] +=1;
            //                 continue;
            //             }
            //         }
            //     }
            //     int colorLabel(0);
            //     for(size_t ii=1; ii<4; ++ii){
            //         if( colorFLag[colorLabel]<colorFLag[ii] ){
            //             colorLabel = ii;
            //         }
            //     }
            //     if(colorLabel==0){
            //         imageFill.at<Vec3b>(i, j).val[0]=0;
            //         imageFill.at<Vec3b>(i, j).val[1]=0;
            //         imageFill.at<Vec3b>(i, j).val[2]=255;
            //     }if(colorLabel==1){
            //         imageFill.at<Vec3b>(i, j).val[0]=0;
            //         imageFill.at<Vec3b>(i, j).val[1]=255;
            //         imageFill.at<Vec3b>(i, j).val[2]=0;
            //     }if(colorLabel==2){
            //         imageFill.at<Vec3b>(i, j).val[0]=255;
            //         imageFill.at<Vec3b>(i, j).val[1]=0;
            //         imageFill.at<Vec3b>(i, j).val[2]=0;
            //     }if(colorLabel==3){
            //         imageFill.at<Vec3b>(i, j).val[0]=255;
            //         imageFill.at<Vec3b>(i, j).val[1]=255;
            //         imageFill.at<Vec3b>(i, j).val[2]=0;
            //     }
            // }