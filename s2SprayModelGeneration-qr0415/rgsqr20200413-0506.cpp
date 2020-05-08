/* ===========
//  * Author: Shepherd Qirong
//  * Date: 2020-04-13 23:52:28
//  * Github: https://github.com/ShepherdQR
//  * LastEditors: Shepherd Qirong
//  * LastEditTime: 2020-05-08 12:55:26
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
#include <pcl/common/common.h>//minmax
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

//using namespace cv;
using namespace std;


bool savePcds = true;

bool processGlobal = true;

bool step2 = true;
bool step1 = false;


bool showOriCloud = false;//memory killer

void index2Color(cv::Mat imageFill, int ii, int jj, int colorLabelP);
int colorLabelF(cv::Vec3b intensity);

string picsFolder = "/home/jellyfish/cpps/FinalPaperSourceCodes/Pics/";
string PicFileName = "LeftBottomRGBY03";


string fileName1 = "/home/jellyfish/cpps/FinalPaperSourceCodes/s1StructurePointCloudSimulation-qr0410";
string scanFiles = fileName1 +"/ScanClouds/2020-04-2403:32:11";

string whichPcd2Load = "/Ori-All-Trans.pcd";
//"/Ori-trans00SL.pcd";
//"/Vox-trans-00SL.pcd"

string projectFolder = "/home/jellyfish/cpps/FinalPaperSourceCodes/s2SprayModelGeneration-qr0415/";


string onceProcessed = "/home/jellyfish/cpps/FinalPaperSourceCodes/s2SprayModelGeneration-qr0415/Processed/Seg10-2020-05-0503:05:21";



float colorList255[10][3]={255, 0, 0,        0, 255, 0,  
          0, 0, 255,        255, 255, 0,
          255, 0, 255,      0, 255, 255,
          255, 97,0,        135, 38, 87,
          210, 180, 140,    0, 199, 140};

float colorList[6][3]={1, 0, 0,
        0, 1, 0,    0, 0, 1,     1, 1, 0,
        0.1, 0.2, 0.3,  0.4, 0.2, 0.3};


float voxXout = 0.1, voxYout = 0.1;
float voxUhat = 0.015, voxVhat = 0.015; // 0.02m

float cameraLocation[4][6]={{0.0, -M_PI_2, 0.0, -2.9, 0.0, 1.03},
                            {0.0, 0.0, 0.0, -0.6, 0.0, 3.1},
                            {0.0, 0.0, 0.0, 0.6, 0.0, 3.1},
                            {0.0, M_PI_2, 0.0, 2.9, 0.0, 1.03}};


void transformAngleAxis( pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, float matrixIn[]);//, int matrixSize = 6


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

        pcl::visualization::PCLVisualizer viewer1("Ori");
        viewer1.setBackgroundColor(255, 255, 255);// 255 *3 = white
        viewer1.addCoordinateSystem();
        
        pcl::visualization::PCLVisualizer viewer3("segmentation");
        viewer3.setBackgroundColor(255, 255, 255);
        viewer3.addCoordinateSystem();

        pcl::visualization::PCLVisualizer viewer4("SLB-1)ori,2)lrf-vox");
        int v41(0), v42(1);
        viewer4.createViewPort(0.0, 0.0, 1.0, 0.5, v42);
        viewer4.createViewPort(0.0, 0.5, 1.0, 1.0, v41);

        viewer4.setBackgroundColor(255, 255, 255);
        viewer4.addCoordinateSystem();

        pcl::visualization::PCLVisualizer viewer5("Spray Model");
        viewer5.setBackgroundColor(255, 255, 255);
        viewer5.addCoordinateSystem();
        

        // pcl::visualization::PCLVisualizer  viewer2("RegionGrowing");
        // int v21(0),v22(1),v23(2),v24(3);
        // viewer2.createViewPort(0.0, 0.0, 0.5, 0.5, v23);//bottomleft=(0,0)
        // viewer2.createViewPort(0.5, 0.0, 1.0, 0.5, v24);
        // viewer2.createViewPort(0.0, 0.5, 0.5, 1.0, v21);
        // viewer2.createViewPort(0.5, 0.5, 1.0, 1.0, v22);
        // viewer2.setBackgroundColor(255, 255, 255);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSLB(new pcl::PointCloud<pcl::PointXYZ>), cloudSLT(new pcl::PointCloud<pcl::PointXYZ>);


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

        if(showOriCloud){
            viewer1.addPointCloud(cloudOri, "ori");
            viewer1.setPointCloudRenderingProperties( pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, "ori");
            viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,1, "ori");
        }
        

        int time_disp=time.toc();
        timeTotal += time_disp; 
        cout<<"To display origional clouds costs time: "<<time_disp<<" ms."<< endl;
        outfile<<"To display origional clouds costs time: "<<time_disp<< " ms."<< endl;
    //---------------


    // 2.2 RegionGrowing --------------in: cloudOri
        time.tic();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOriVox(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloudOri, *cloudOriVox);
        
        // Normally we should do voxgrid or we process the vox-pcd using filtering, here to do things more generally, I spent some time doing voxgrid. However, donnot know the reason why it doesnot work out. The fucking stupid code! The FUCKING out of memory really bothered me for quite a long time. Then I gave it up, and leave this.


        pcl::search::Search<pcl::PointXYZ>::Ptr tree_ori_filter = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud <pcl::Normal>::Ptr normals_ori_filter (new pcl::PointCloud <pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_ori_filter;
        normal_ori_filter.setSearchMethod (tree_ori_filter);
        normal_ori_filter.setInputCloud ( cloudOriVox );
        normal_ori_filter.setKSearch (50);
        normal_ori_filter.compute (*normals_ori_filter);
        cout<<"  Gotcha the normals of the cloud after filtering."<<endl;
        outfile<<"  Gotcha the normals of the cloud after filtering."<<endl;

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg_ori_filter;
        reg_ori_filter.setMinClusterSize (2500);//800 box
        reg_ori_filter.setMaxClusterSize (3000000);
        reg_ori_filter.setSearchMethod (tree_ori_filter);
        reg_ori_filter.setNumberOfNeighbours (50);
        reg_ori_filter.setInputCloud (cloudOriVox);
        reg_ori_filter.setInputNormals (normals_ori_filter);
        reg_ori_filter.setSmoothnessThreshold (10.0 / 180.0 * M_PI);//senstive
        reg_ori_filter.setCurvatureThreshold (4.0);//4

        std::vector <pcl::PointIndices> clusterOri;
        reg_ori_filter.extract (clusterOri);

        // no idea why the following codes run error.
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr regiongrowing_cloudcolored_target = reg_ori_filter.getColoredCloud (); 
        //viewer3.addPointCloud(regiongrowing_cloudcolored_target, "cloud_regiongrowing_v14" );

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

        
        for(size_t i=0; i<10; ++i){
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
            float cr=colorList255[i][0]/255, cg = colorList255[i][1]/255, cb = colorList255[i][2]/255;
            cout << cr << ", " << cg << ", " << cb << endl;

            viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, cr,cg,cb, "cloudv"+ ss_i.str());

            cout << "  the " << i << "th cluster's point number is: " <<cloudMain->width * cloudMain->height << endl;
            outfile << "  the " << i << "th cluster's point number is: " <<cloudMain->width * cloudMain->height << endl;

            if(processGlobal==false){
                if((i==0) | (i==1)){
                    *cloudSLB = *cloudSLB + *cloudMain;
                }
                if((i==2) | (i==3)){
                    *cloudSLT = *cloudSLT + *cloudMain;
                }
            }

            
        }

        if(savePcds & (processGlobal == false)){
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


    // this section can be part of the whole process, the rgs clusters need to be classed with location.
    // in this step, I process unstructed pcd.
    if(step2){
    //============ 3  figure process 
    // 3.1 load and preprocess, just to fill the boundaries.
        string imageName( picsFolder + PicFileName +  ".png" );
        cv::Mat iIn= cv::imread( imageName ); 
        //default = IMREAD_COLORIM.   READ_GRAYSCALE

        //namedWindow( "Display window", WINDOW_AUTOSIZE ); 
        //imshow( "Display window", iIn );  

        int nRows = iIn.rows, nCols = iIn.cols;
        //cout << nRows << endl;
        cv::Mat imageFill= iIn.clone();
        //cv::Mat imageFillCopy= imageFill.clone();

        int colorLabelPList[nRows][nCols]={0};
        for(size_t  i = 0; i < nRows; ++i){
            for(size_t j = 0; j < nCols; ++j){
                cv::Vec3b intensity = imageFill.at<cv::Vec3b>(i, j);
                // float blue1 = intensity.val[0];
                // float green1 = intensity.val[1];
                // float red1 = intensity.val[2];
                int colorLabel = colorLabelF( intensity);
                
                if( colorLabel == 4){
                    int whichColor[2][2]={{4,nRows},{4,nRows}};
                    int colorLabelP=4;

                    int s11=0, s12=0;
                    for(size_t indi = 1; indi < nCols/3; ++indi){
                        if(s11 || s12)  break;
                        if(((int)(i-indi)> 0) & (s11==0)){
                            cv::Vec3b intensityP1 = imageFill.at<cv::Vec3b>((int)(i-indi), j);
                            int colorLabelP1 = colorLabelF( intensityP1 );
                            if( colorLabelP1 != 4 ){
                                whichColor[0][0]= colorLabelP1;
                                whichColor[0][1]= -1 * indi;
                                s11 =1;
                            } 
                        }
                        if((i+indi) < nRows){
                            cv::Vec3b intensityP2 = imageFill.at<cv::Vec3b>((i+indi), j);
                            int colorLabelP2 = colorLabelF( intensityP2 );
                            if(  colorLabelP2 != 4 & (s12==0)){
                                whichColor[1][0]= colorLabelP2;
                                whichColor[1][1]= indi;
                                s12 = 1;
                            } 
                        }  
                    }
                    if((whichColor[0][1] * whichColor[0][1]) < (whichColor[1][1] * whichColor[1][1]))
                        colorLabelP = whichColor[0][0];
                    else colorLabelP = whichColor[1][0];

                    colorLabelPList[i][j]=colorLabelP;
                    index2Color(imageFill, i, j, colorLabelP);
                    // imageFill.at<cv::Vec3b>(i, j).val[0]=255;
                    // imageFill.at<cv::Vec3b>(i, j).val[1]=255;
                    // imageFill.at<cv::Vec3b>(i, j).val[2]=255;
                }
            }
        }
        //imshow("imageFill",  imageFill);
        //imwrite(picsFolder + PicFileName +  "Narrow.png", imageFill);


    // ========  4 region preprocess ==
    //  4.1  pcd pass
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2slb(new pcl::PointCloud<pcl::PointXYZ>), cloudLoadCur(new pcl::PointCloud<pcl::PointXYZ>);

        int loadList[]={2,5};
        for(size_t i =0; i<(sizeof(loadList)/sizeof(loadList)[0]); ++i){
            std::stringstream ss_i;
	        ss_i  << loadList[i];
            pcl::io::loadPCDFile(onceProcessed +"/segmentation" +ss_i.str()+ ".pcd",*cloudLoadCur);
            *cloud2slb = *cloud2slb + *cloudLoadCur;
        }


    //  4.2 trans   the slt is uncompleted and cannot be processed perfect.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2slbTrans(new pcl::PointCloud<pcl::PointXYZ>);
        float matrixCurIn2[6]={0};
            for(size_t im = 0; im< 6; ++im){
                matrixCurIn2[im]= -1 * cameraLocation[0][im];
            }
        float matrixCurIn3[6]={0,0,0,matrixCurIn2[3],matrixCurIn2[4],matrixCurIn2[5]};
        transformAngleAxis(cloud2slb, cloud2slbTrans, matrixCurIn3);

        float matrixCurIn4[6]={matrixCurIn2[0],matrixCurIn2[1],matrixCurIn2[2], 0,0,0};
        transformAngleAxis(cloud2slbTrans, cloud2slbTrans, matrixCurIn4);
        //viewer4.updatePointCloud(cloud2slbTrans, "slb");

    //  5 vox the ori pcd
    //  5.1  minmax of the pcd..
        vector<int> listCornor(6,-1);
        pcl::PointXYZ pmin;
        pcl::PointXYZ pmax;
        pcl::getMinMax3D(*cloud2slbTrans,pmin,pmax);
        cout << "pmin: (" << pmin.x <<", " << pmin.y <<", " << pmin.z << ")." << endl
        << "pax: (" << pmax.x <<", " << pmax.y <<", " << pmax.z << ")." << endl;
        outfile << "pmin: (" << pmin.x <<", " << pmin.y <<", " << pmin.z << ")." << endl
        << "pax: (" << pmax.x <<", " << pmax.y <<", " << pmax.z << ")." << endl;

        for(size_t i=0; i<cloud2slbTrans->size(); ++i){
            listCornor[0]=(cloud2slbTrans->points[i].x == pmin.x)?i:listCornor[0];
            listCornor[1]=(cloud2slbTrans->points[i].y == pmin.y)?i:listCornor[1];
            listCornor[2]=(cloud2slbTrans->points[i].z == pmin.z)?i:listCornor[2];
            listCornor[3]=(cloud2slbTrans->points[i].x == pmax.x)?i:listCornor[3];
            listCornor[4]=(cloud2slbTrans->points[i].y == pmax.y)?i:listCornor[4];
            listCornor[5]=(cloud2slbTrans->points[i].z == pmax.z)?i:listCornor[5];
        }
        for(int i=0;i<6;++i){
            cout << "  " << listCornor[i] << ": (" << cloud2slbTrans->points[listCornor[i]].x <<", " << cloud2slbTrans->points[listCornor[i]].y <<", " <<  cloud2slbTrans->points[listCornor[i]].z << ")." << endl;
            outfile << "  " << listCornor[i] << ": (" << cloud2slbTrans->points[listCornor[i]].x <<", " << cloud2slbTrans->points[listCornor[i]].y <<", " <<  cloud2slbTrans->points[listCornor[i]].z << ")." << endl;
        }


    // 5.2 vox setting
        int numY = ceil((pmax.y - pmin.y) / voxUhat)+1;
        int numX = ceil((pmax.x - pmin.x) / voxVhat)+1;
        float voxY = (pmax.y - pmin.y) / (numY-1);
        float voxX = (pmax.x - pmin.x) / (numX-1);

        vector<float> xList(numX, 0.0);//
        for(size_t i = 0; i< numX; ++i){
            xList[i] = pmax.x - i * voxX;
        }
        vector<float> yList(numY, 0.0);//
        for(size_t i = 0; i< numY; ++i){
            yList[i] = pmax.y -  i * voxY;
        }
        cout << " Lattie length: " << voxY << ", " << voxX <<  endl;
        outfile << " Lattie length: " << voxY << ", " << voxX <<  endl;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudVox01(new pcl::PointCloud<pcl::PointXYZI>);
        cloudVox01->width = numY;
        cloudVox01->height = numX;
        cloudVox01->points.resize(cloudVox01->width * cloudVox01->height);

        int numberPointsOri = cloud2slbTrans->width * cloud2slbTrans->height;
        cout <<" point number is voxed to: " << 
        cloud2slbTrans->width <<", "<< cloud2slbTrans->height << ", "<< numberPointsOri << " -> " << cloudVox01->width <<", "<< cloudVox01->height << ", "<< cloudVox01->width * cloudVox01->height << endl;
        outfile <<" point number is voxed to: " << 
        cloud2slbTrans->width <<", "<< cloud2slbTrans->height << ", "<< numberPointsOri << " -> " << cloudVox01->width <<", "<< cloudVox01->height << ", "<< cloudVox01->width * cloudVox01->height << endl;


    // 5.3 vox 01, just to downsize
        // here at(i,j), both from max to min, which is, after transform back, from the frot-top of the tank.
        for(size_t i = 0; i< numY ; ++i){
            for(size_t ii = 0; ii<numX ; ++ii){
                cloudVox01->at(i,ii).x = xList[ii];
                cloudVox01->at(i,ii).y = yList[i];
                cloudVox01->at(i,ii).z = -0.35;
                cloudVox01->at(i,ii).intensity = 0.0;

            }// attention, the order must be: at(numV, numU), first count the number in one row, then count the number of rows.
        }//cout << cloudVox01->at(3, 6).x << endl;
         if(savePcds){
            pcl::io::savePCDFile (processedPath +"/vox-SLB1.pcd",*cloudVox01);
        }

        for(size_t i=0; i<numberPointsOri;++i ){
            int orderX = ceil(( pmax.x -cloud2slbTrans->points[i].x - voxX/2)/voxX );
            int orderY = ceil(( pmax.y -cloud2slbTrans->points[i].y - voxY/2)/voxY );
            orderY = (orderY >= numY)? (numY-1) : orderY;
            orderX = (orderX >= numX)? (numX-1) : orderX;
            cloudVox01->at(orderY, orderX).z = cloud2slbTrans->points[i].z;
            cloudVox01->at(orderY, orderX).intensity += 1.0;
        }
        if(savePcds){
            pcl::io::savePCDFile (processedPath +"/vox-SLB.pcd",*cloudVox01);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVox01disp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloudVox01, *cloudVox01disp);

        viewer4.addPointCloud(cloud2slbTrans, "cloudslbTrans", v41);
        viewer4.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, "cloudslbTrans", v41);
        viewer4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloudslbTrans", v41);
        viewer4.setCameraPosition((pmax.x + pmin.x)/2, (pmax.y + pmin.y)/2, 0 *(pmax.z + pmin.z)/2, (pmax.x + pmin.x)/2, (pmax.y + pmin.y)/2, (pmax.z + pmin.z)/2,  1.0,  0.0,  0.0);//pox_x,...; view_x,...; up_x,...

        viewer4.addPointCloud(cloudVox01disp, "cloudslbVox", v42);
        viewer4.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, "cloudslbVox", v42);
        viewer4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0, 0, "cloudslbVox", v42);


    // 5.4 vox 02 generate the lattices
            // notice that we count not the points but the gaps here. 
        int numYout = (pmax.y - pmin.y) / voxYout;
        int numXout = (pmax.x - pmin.x) / voxXout;


        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloudVox02(new pcl::PointCloud<pcl::PointXYZRGBL>);
        cloudVox02->width = numYout;
        cloudVox02->height = numXout;
        cloudVox02->points.resize(cloudVox02->width * cloudVox02->height);
        cout <<" Lattice number is: " <<
        cloudVox02->width <<", "<< cloudVox02->height << ", "<< cloudVox02->width * cloudVox02->height << endl;
        outfile <<" Lattice number is: " <<
        cloudVox02->width <<", "<< cloudVox02->height << ", "<< cloudVox02->width * cloudVox02->height << endl;

        int nn(0);
        float outPoints[numYout * numXout][8]={0};//xyz,rgb,label+bool
        for(size_t i = 0; i< numYout ; ++i){
            for(size_t ii = 0; ii<numXout ; ++ii){
                float x = pmax.x-voxXout/2 - ii * voxXout ;
                float y = pmax.y-voxYout/2 - i * voxYout;
                int latticeCornerX = (pmax.x - x)/ voxX;//voxX
                int latticeCornerY = (pmax.y - y)/ voxY;


                int near4[4]={0}, sumZ(0), notZero(0);
                int latticeCornerYnext = (latticeCornerY+1 >= numY)? (numY-1) : (latticeCornerY+1);
                int latticeCornerXnext = (latticeCornerX+1 >= numX)? (numX-1) : (latticeCornerX+1);

                near4[0] = cloudVox01->at(latticeCornerY, latticeCornerX).z;
                near4[1] = cloudVox01->at(latticeCornerY, latticeCornerXnext).z;
                near4[2] = cloudVox01->at(latticeCornerYnext, latticeCornerXnext).z;
                near4[3] = cloudVox01->at(latticeCornerYnext, latticeCornerX).z;
                    //anticlokewise

                for(int ij=0; ij<4; ++ij){
                    if(near4[ij]!=0)
                        ++ notZero;
                    sumZ += near4[ij];
                }
                float z = (notZero == 0)? 0.0 : (sumZ/notZero);// this equation controls the cover mode, whether to over cover, or on contray.
                
                cloudVox02->at(i,ii).x = x;
                cloudVox02->at(i,ii).y = y;
                cloudVox02->at(i,ii).z = z;

                
                int indexRow = (pmax.x - x)/(pmax.x-pmin.x) * nRows;// the range is (pmax.x-pmin.x). I just sitting on the bed and all of a sudden, I thought this is the key point I missed. Then after a few seconds, all worked out!
                int indexCol = (pmax.y - y)/(pmax.y-pmin.y) * nCols;
                float colorR = imageFill.at<cv::Vec3b>(indexRow, indexCol).val[2]/255;
                float colorG = imageFill.at<cv::Vec3b>(indexRow, indexCol).val[1]/255;
                float colorB = imageFill.at<cv::Vec3b>(indexRow, indexCol).val[0]/255;
                //if(ii ==0) cout << colorR << ", " << colorG << ", " << colorB << endl;
                
                

                cloudVox02->at(i,ii).r = colorR;
                cloudVox02->at(i,ii).g = colorG;
                cloudVox02->at(i,ii).b = colorB;
                cloudVox02->at(i,ii).label =colorLabelPList[i][ii];

                stringstream ssn;
                ssn << ++nn; 
                string name="cube"+ssn.str();
                if((colorLabelPList[i][ii] !=4) & (z < -0.35) ){
                    viewer5.addCube (x- voxXout/2, x+voxXout/2,y-voxYout/2, y+voxYout/2, z, z+0.01, colorR, colorG, colorB, name);
                    outPoints[i+i*ii+ii][7]=1;
                    outPoints[i+i*ii+ii][6]=colorLabelPList[i][ii];
                    outPoints[i+i*ii+ii][5]=colorB;
                    outPoints[i+i*ii+ii][4]=colorG;
                    outPoints[i+i*ii+ii][3]=colorR;
                }
            }
        }



        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVox02Trans0(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloudVox02, *cloudVox02Trans0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVox02Trans1(new pcl::PointCloud<pcl::PointXYZ>);
        float matrixCurIn6[6]={0};
        for(size_t im = 0; im< 6; ++im){
            matrixCurIn6[im]=   cameraLocation[0][im];
        }
        
        transformAngleAxis(cloudVox02Trans0, cloudVox02Trans1, matrixCurIn6);
        for(size_t i = 0; i< numYout ; ++i){
            for(size_t ii = 0; ii<numXout ; ++ii){

                outPoints[i+i*ii+ii][2]=cloudVox02Trans1->at(i,ii).z;
                outPoints[i+i*ii+ii][1]=cloudVox02Trans1->at(i,ii).y;
                outPoints[i+i*ii+ii][0]=cloudVox02Trans1->at(i,ii).x; 
            }
        }

        cout << "spray model in " << voxXout << "x" << voxXout << endl;
        outfile << "spray model in " << voxXout << "x" << voxXout << endl;
        outfile << "x, y, z, r, g, b, colorClass" << endl;
        for(size_t i=0; i<numYout * numXout; ++i ){
            if(outPoints[i][7]==1){
                for(size_t j=0; j<6; ++j ){
                    outfile <<  outPoints[i][j] << ", ";
                }
                outfile <<  outPoints[i][7] << endl;
            }
        }
        







        //viewer5.addPointCloud(cloudVox02, "cloudslbVox" );
        //viewer5.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, "cloudslbVox" );
        viewer5.setCameraPosition((pmax.x + pmin.x)/2, (pmax.y + pmin.y)/2, 0 *(pmax.z + pmin.z)/2, (pmax.x + pmin.x)/2, (pmax.y + pmin.y)/2, (pmax.z + pmin.z)/2,  1.0,  0.0,  0.0);
        

        if(savePcds){
            pcl::io::savePCDFile (processedPath +"/final-spray-SLB.pcd",*cloudVox02);
        } 


        
        


    }

   








    






    
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











    // //cv::Mat iOut;
    // //int rWidth = cloudCur->width , rHeight = cloudCur->height;
    // //cv::resize(imageFill, iOut, cv::Size( rWidth, round(rHeight*0.5)), INTER_NEAREST );//INTER_LINEAR  INTER_AREA
    // // imshow("imageOut", iOut);










//=============================

















    // float lattieX = 0.0301336, lattieY = 0.1;



    // float ky= nRows /(planeSLB[3] - planeSLB[2]   ), kx =  nCols /(cloudCur->width );// based on the search we use 2 and 3 clusters
    // float pUVRange[4]={cloudCur->at((cloudCur->width)-1, planeSLB[3]+1 ).x,cloudCur->at((cloudCur->width)-1, planeSLB[3]+1 ).y, cloudCur->at(0, planeSLB[2]).x, cloudCur->at(0, planeSLB[2] ).y };
    // cout << "LT and RB:" << 
    // pUVRange[0] << "; " << pUVRange[1] << "; " <<
    // pUVRange[2] << ", " << pUVRange[3] << ", " << endl;
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
    //         if(imageFill.at<cv::Vec3b>(ati, atj).val[2]>145 ){
    //             colorR=1;
    //         }
    //         if(imageFill.at<cv::Vec3b>(ati, atj).val[1]>145 ){
    //             colorG=1;
    //         }
    //         if(imageFill.at<cv::Vec3b>(ati, atj).val[0]>145 ){
    //             colorB=1;
    //         }
    //         //double colorR=imageFill.at<cv::Vec3b>(ati, atj).val[2]/255, colorG=imageFill.at<cv::Vec3b>(ati, atj).val[1]/255,colorB=imageFill.at<cv::Vec3b>(ati, atj).val[0]/255;

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


void index2Color(cv::Mat imageFill, int ii, int jj,  int colorLabelP){
    //cv::Vec3b intensity = imageFill.at<cv::Vec3b>(ii, jj);
    if(colorLabelP==0){
        imageFill.at<cv::Vec3b>(ii, jj).val[0]=0;
        imageFill.at<cv::Vec3b>(ii, jj).val[1]=0;
        imageFill.at<cv::Vec3b>(ii, jj).val[2]=255;
    }else if(colorLabelP==1){
        imageFill.at<cv::Vec3b>(ii, jj).val[0]=0;
        imageFill.at<cv::Vec3b>(ii, jj).val[1]=255;
        imageFill.at<cv::Vec3b>(ii, jj).val[2]=0;
    }else if(colorLabelP==2){
        imageFill.at<cv::Vec3b>(ii, jj).val[0]=255;
        imageFill.at<cv::Vec3b>(ii, jj).val[1]=0;
        imageFill.at<cv::Vec3b>(ii, jj).val[2]=0;
    }else if(colorLabelP==3){
        imageFill.at<cv::Vec3b>(ii, jj).val[0]=0;
        imageFill.at<cv::Vec3b>(ii, jj).val[1]=255;
        imageFill.at<cv::Vec3b>(ii, jj).val[2]=255;
    }
}



int colorLabelF(cv::Vec3b intensity){

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