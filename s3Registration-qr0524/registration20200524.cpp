/* ===========
//  * Author: Shepherd Qirong
//  * Date: 2020-05-24 17:15:43
//  * Github: https://github.com/ShepherdQR
//  * LastEditors: Shepherd Qirong
//  * LastEditTime: 2020-05-25 07:46:45
//  * Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
*/

#include <boost/filesystem.hpp>//
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>


#include <pcl/common/transforms.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>


#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

using namespace std;


bool useRawPcds = false;
float voxSize = 0.04;

bool useSac = false;
bool randomTransTar = true;
int seedRandom = 66;
bool savePcds = true;
string onceVox = "20200525-03:23:48";



string projectFolder = "/home/jellyfish/cpps/FinalPaperSourceCodes/s3Registration-qr0524/";

int pcdNumber = 3;
string pcdFolderName[] = {"Tank1-T99A-480-20200516-03:29:12", "Tank2-2A5-480-2020-05-2410:25:38", "Tank3-T90A-480-2020-05-2410:41:56"},
pcdFolderNameL = "/home/jellyfish/cpps/FinalPaperSourceCodes/s1StructurePointCloudSimulation-qr0410/ScanClouds/",
pcdFolderNameR = "/Vox-All-TransMeaning.pcd";


string voxPcds[] = {"vox-Tank1-T99A", "vox-Tank2-2A5","vox-Tank3-T90A"};




void randomList(float trans6Axis[], int numIn, float rangeL, float rangeR, int seedIn );

void transformAngleAxis( pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, float matrixIn[]);//, int matrixSize = 6

void print4x4Matrix (const Eigen::Matrix4d & matrix){
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void compute_normal( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, std::vector<int> indices_in_real,   const pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals, const float RadiusSearch);
void FeatureNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, const float RadiusSearch, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

void feature_fpfh( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt,pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals, double fpfh_radius_search,pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt);

int main( int argc, char** argv ){

    // ==  1    initialization
    // =================================
    // 1.1  time & log initialization--------
        pcl::console::TicToc time;
        time.tic();
        int timeTotal(0);

        time_t timeLocal99 = std::time(NULL);
        char timeCurrent99[64];
        strftime(timeCurrent99, sizeof(timeCurrent99), "%Y%m%d-%H:%M:%S",localtime(&timeLocal99));
        string timeCurrent100 = timeCurrent99;
        string processedPath = projectFolder + "voxelPcds/" + timeCurrent100;
        boost::filesystem::create_directories(processedPath);

        string fileName99 = processedPath + "/"+ timeCurrent100 + ".log";
        ofstream outfile( fileName99);//c++11 write like this!.c_str
        outfile << timeCurrent99 << endl;
    //---------------------------


    // 1.2  general initialization----------------
        int viewerNumber = 3;
        string windowNames[viewerNumber] = {"T99A", "2A5", "T90A"};
        pcl::visualization::PCLVisualizer viewer[viewerNumber];
        for(size_t i=0; i<viewerNumber; ++i){
            viewer[i].setBackgroundColor(255, 255, 255);// 255 *3 = white
            viewer[i].addCoordinateSystem();
            viewer[i].setWindowName(windowNames[i]);
        }

        // viewer1.
        // viewer1.add

        // pcl::visualization::PCLVisualizer viewer2("Ori-2");
        // viewer2.setBackgroundColor(255, 255, 255);
        // viewer2.addCoordinateSystem();

        // pcl::visualization::PCLVisualizer viewer3("Ori-3");
        // viewer3.setBackgroundColor(255, 255, 255);
        // viewer3.addCoordinateSystem();


        Eigen::Matrix4d	tarTransMatrixSACIA = Eigen::Matrix4d::Identity (), tarTransMatrixICP = Eigen::Matrix4d::Identity ();
        
        
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        
        



        int time_visual_init=time.toc();
        timeTotal += time_visual_init; 
        cout<<"Visualization Initialization costs time: "<<time_visual_init<<" ms."<< endl;
        outfile<<"Visualization Initialization costs time: "<<time_visual_init<< " ms."<< endl;
    //---------------------
// ==============================

    // load pcds
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTem(new pcl::PointCloud<pcl::PointXYZ>), cloudTar(new pcl::PointCloud<pcl::PointXYZ>), cloudCur(new pcl::PointCloud<pcl::PointXYZ>), cloudCurVox(new pcl::PointCloud<pcl::PointXYZ>),cloudTarTrans(new pcl::PointCloud<pcl::PointXYZ>);

        double regScores[pcdNumber]={-1}, ave[pcdNumber]={-1};
        for (size_t i = 0; i < pcdNumber; ++i){
            string curPcdPath = pcdFolderNameL + pcdFolderName[i] + pcdFolderNameR;
            if(!useRawPcds){
                curPcdPath = projectFolder + "/voxelPcds/" + onceVox + "/" + voxPcds[i] + "-"  + onceVox  +".pcd";
            }

            
            pcl::io::loadPCDFile(curPcdPath, *cloudCur);
            outfile << "load pcd files from: " << curPcdPath << endl; 
            pcl::copyPointCloud(*cloudCur, *cloudCurVox);

            if(useRawPcds){
                cloudCurVox->clear();
                pcl::VoxelGrid<pcl::PointXYZ> sor12;
                sor12.setInputCloud(cloudCur);
                sor12.setLeafSize (voxSize, voxSize, voxSize);
                sor12.filter(*cloudCurVox);
                cout << "Origional point number is: " << cloudCur->points.size() << "\nCurrent voxel point number is: " << cloudCurVox->points.size() << endl;//
                outfile << "Origional point number is: " << cloudCur->points.size() << "\nCurrent voxel point number is: " << cloudCurVox->points.size() << endl;//            
            }

            cloudTem->clear();
            pcl::copyPointCloud(*cloudCurVox, *cloudTem);


            int freedom6(6);
            float trans6Axis[freedom6];
            Eigen::Matrix4d transMatrixOri = Eigen::Matrix4d::Identity ();
            if(i==0){
                //pcl::copyPointCloud(*cloudCurVox, *cloudTar);
                if(randomTransTar){
                    
                    randomList(trans6Axis, freedom6, -1,  1, seedRandom );
                    for(int i=0; i< freedom6; ++i){
                        cout << trans6Axis[i] << endl;
                        outfile << trans6Axis[i] << endl;
                    }
                    Eigen::Matrix3d transMatrixOriX3 = Eigen::Matrix3d::Identity ();
                    transMatrixOriX3(1,1) = cos(trans6Axis[3]);
                    transMatrixOriX3(1,2) = sin(trans6Axis[3]);
                    transMatrixOriX3(2,1) = -sin(trans6Axis[3]);
                    transMatrixOriX3(2,2) = cos(trans6Axis[3]);

                    Eigen::Matrix3d transMatrixOriY3 = Eigen::Matrix3d::Identity ();
                    transMatrixOriX3(0,0) = cos(trans6Axis[4]);
                    transMatrixOriX3(0,2) = -sin(trans6Axis[4]);
                    transMatrixOriX3(2,0) = sin(trans6Axis[4]);
                    transMatrixOriX3(2,2) = cos(trans6Axis[4]);

                    Eigen::Matrix3d transMatrixOriZ3 = Eigen::Matrix3d::Identity ();
                    transMatrixOriX3(0,0) = cos(trans6Axis[5]);
                    transMatrixOriX3(0,1) = sin(trans6Axis[5]);
                    transMatrixOriX3(1,0) = -sin(trans6Axis[5]);
                    transMatrixOriX3(1,1) = cos(trans6Axis[5]);

                    Eigen::Matrix3d transMatrixOriXYZ3 = Eigen::Matrix3d::Identity ();
                    transMatrixOriXYZ3  =  transMatrixOriZ3 *transMatrixOriY3 *transMatrixOriX3 * transMatrixOriXYZ3;
                   
                    transMatrixOri.block<3,3>(0,0) = transMatrixOriXYZ3;
                    transMatrixOri(0,3) = trans6Axis[0];
                    transMatrixOri(1,3) = trans6Axis[1];
                    transMatrixOri(2,3) = trans6Axis[2];                    
                }
            
                print4x4Matrix(transMatrixOri);
                for(size_t j = 0; j< 4; ++j){
                    outfile << transMatrixOri(j,0) <<", " << transMatrixOri(j,1) <<", " << transMatrixOri(j,2) << ", " << transMatrixOri(j,3) << endl;
                }

                transformAngleAxis(cloudCurVox,cloudTar, trans6Axis);

                //pcl::transformPointCloud (*cloudCurVox,*cloudTar, transMatrixOri);
                
                
                if(savePcds & (!useRawPcds)){
                    pcl::io::savePCDFile(projectFolder + "/voxelPcds/" + timeCurrent100 + "/TargetTrans-"  + timeCurrent100  +".pcd", *cloudTar);
                }
            }

            
            viewer[i].addPointCloud(cloudTem, "cloudTem");
            viewer[i].setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 3, "cloudTem");
            viewer[i].setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0, 0, "cloudTem");

            viewer[i].addPointCloud(cloudTar, "cloudTar");
            viewer[i].setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 3, "cloudTar");
            viewer[i].setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1, 0, "cloudTar");

        
        // ---sac 
        // normal---target & template
            //std::vector<int> indices_target_real, indices_template_real;
            double scoreSACIA(0);
            if(useSac){
            pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud< pcl::Normal>), template_normals(new pcl::PointCloud< pcl::Normal>);
            FeatureNormal(cloudTem, 0.1, template_normals);
            FeatureNormal(cloudTar, 0.1, target_normals);
           

            pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhsTar(new pcl::PointCloud<pcl::FPFHSignature33>()),fpfhsTem(new pcl::PointCloud<pcl::FPFHSignature33>());
        
            feature_fpfh( cloudTar,target_normals, 0, fpfhsTar);
            feature_fpfh( cloudTem,template_normals, 0, fpfhsTem);

            sac_ia.setInputSource(cloudTem);//template
            sac_ia.setInputTarget(cloudTar);
            sac_ia.setSourceFeatures( fpfhsTem );
            sac_ia.setTargetFeatures( fpfhsTar );
        
            sac_ia.setNumberOfSamples( 15 );//60
            sac_ia.setCorrespondenceRandomness( 10 );//120
            sac_ia.setEuclideanFitnessEpsilon(0.01);
            sac_ia.setTransformationEpsilon(0.01);
            sac_ia.setRANSACIterations(180);
            sac_ia.setMaximumIterations(30);
            sac_ia.align(*cloudTarTrans);
            cout << "hi" << endl;
            tarTransMatrixSACIA =sac_ia.getFinalTransformation ().cast<double>();
            scoreSACIA = sac_ia.getFitnessScore();
            
        }
        if(!useSac){
            pcl::copyPointCloud(*cloudTar, *cloudTarTrans);
        }

            
            icp.setInputSource (  cloudTarTrans );
            icp.setInputTarget (  cloudTem ); 
            icp.setEuclideanFitnessEpsilon( 0.001 );
            icp.setMaximumIterations ( 600 ); 
            icp.align ( *cloudTarTrans );
            tarTransMatrixICP = icp.getFinalTransformation ().cast<double>();
            double scoreICP = icp.getFitnessScore();
            print4x4Matrix( tarTransMatrixICP );

            if(savePcds ){
                pcl::io::savePCDFile(projectFolder + "/voxelPcds/" + timeCurrent100 + "/Reg-" + voxPcds[i] + "-" + timeCurrent100  +".pcd", *cloudTarTrans);
            }

            //regScores[i] = scoreSACIA + scoreICP;
            //cout << "sca-ia score: " << scoreSACIA << ", icp score: " << scoreICP << ", and sums: " << regScores[i] << endl;
            //outfile << "sca-ia score: " << scoreSACIA << ", icp score: " << scoreICP << ", and sums: " << regScores[i] << endl;

            regScores[i] =  pow(scoreICP, 0.5) / cloudTem->size();
            cout << regScores[i] << endl;
            outfile << regScores[i] << endl;


            pcl::transformPointCloud(*cloudTar, *cloudTarTrans, tarTransMatrixICP);
            //viewer[i].updatePointCloud(cloudTarTrans, "cloudTar");
            viewer[i].addPointCloud(cloudTarTrans, "cloudTarTrans");
            viewer[i].setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 3, "cloudTarTrans");
            viewer[i].setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,1,  "cloudTarTrans");


        //=====differ
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            int kIn = 10;
            vector<float> pointDistance(10);
            kdtree.setInputCloud(cloudTarTrans);

            vector<int> outIndex(cloudTem->points.size());
            vector<float> outDistance(cloudTem->points.size(), 6);
            for(size_t j=0; j<  cloudTem->points.size() ; ++j){
                vector<int> pointIndex(kIn);
                kdtree.nearestKSearch (cloudTem->points[j] , kIn, pointIndex, pointDistance) ;

                //cout << "(" << cloudTarTrans->points[j].x << ", " << cloudTarTrans->points[j].y << ", " << cloudTarTrans->points[j].z << ")" << endl;
                for (size_t i = 0; i < pointIndex.size (); ++i){
                    float distance2(0), distance1(0);
                    distance2 = pow((cloudTem->points[j].x -   cloudTarTrans->points[ pointIndex[i] ].x), 2)+ pow((cloudTem->points[j].y -   cloudTarTrans->points[ pointIndex[i] ].y), 2)+ pow((cloudTem->points[j].z -   cloudTarTrans->points[ pointIndex[i] ].z), 2);
                    distance1 = pow(distance2, 0.5);
                    if(distance1 < outDistance[j] ){
                        outDistance[j] = distance1;
                        outIndex[j] =  pointIndex[i];
                    }
                }  
            }

            for(size_t j=0; j<  cloudTem->points.size() ; ++j){
                ave[i] += outDistance[j];
            }
            ave[i] = ave[i] /(cloudTem->points.size());
            cout << ave[i] << endl;
            outfile << ave[i] << endl;



            if(useRawPcds & savePcds){
                pcl::io::savePCDFile(projectFolder + "/voxelPcds/" + timeCurrent100 + "/" + voxPcds[i] + "-"  + timeCurrent100  +".pcd", *cloudCurVox);
            }


        }

   

    while (!viewer[0].wasStopped()){
        viewer[0].spinOnce();
    }
    return 0;
}


void randomList(float trans6Axis[], int numIn, float rangeL, float rangeR, int seedIn){
    default_random_engine engienR(seedIn);
    uniform_real_distribution<float> u(0, (rangeR - rangeL));
    for(size_t i = 0; i< numIn; ++i){//(sizeof(trans6Axis)/sizeof(trans6Axis[0]))
        trans6Axis[i] = u(engienR);
    }
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



void FeatureNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, const float RadiusSearch, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloudIn);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(kdtree);
    ne.setRadiusSearch(RadiusSearch);//LIKE 0.03m
    ne.compute(*cloud_normals);
    cout << "Normal ok" << endl;
    cout << "(nx, ny, nz, curvature)= "<< cloud_normals->points[0].normal_x << ", "<< cloud_normals->points[0].normal_y << ", "<< cloud_normals->points[0].normal_z << ", "<< cloud_normals->points[0].curvature << "."<<endl;
}






void feature_fpfh( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt,pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals, double fpfh_radius_search,pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt){
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
    fpfh_tgt.setInputCloud(cloud_tgt);
    fpfh_tgt.setInputNormals(cloud_tgt_normals);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<pcl::PointXYZ>);
    fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
    fpfh_tgt.setRadiusSearch( fpfh_radius_search ); 
    //fpfh_tgt.setKSearch( 20 );
    //fpfh_tgt.setRadiusSearch( 10 );
    fpfh_tgt.compute(*fpfhs_tgt);
}




void compute_normal( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, std::vector<int> indices_in_real,   pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals, const float RadiusSearch  ){
    //pcl::removeNaNFromPointCloud(*cloud_tgt, *cloud_tgt, indices_in_real);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
    ne_tgt.setInputCloud(cloud_tgt);
    pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
    ne_tgt.setSearchMethod(tree_tgt);
    //ne_tgt.setRadiusSearch( RadiusSearch );
    ne_tgt.setKSearch( 20 );
    ne_tgt.compute(*cloud_tgt_normals);
}