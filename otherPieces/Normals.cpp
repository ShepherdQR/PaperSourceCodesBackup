/* ===========
//  * Author: Shepherd Qirong
//  * Date: 2020-03-07 22:44:30
//  * Github: https://github.com/ShepherdQR
//  * LastEditors: Shepherd Qirong
//  * LastEditTime: 2020-03-24 02:08:37
//  * Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
*/







//-------------------------pcl
#include <pcl/console/parse.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/keypoints/sift_keypoint.h> //20200323
#include <pcl/visualization/cloud_viewer.h>


#include <fstream>
#include <iostream>
#include <string>
#include <time.h>
#include <vector>
using namespace std;
// --------------------



int voxelGrid(0);
// ---------------------------- Filter --- begin
void FilterVoxelGrid(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudIn, const float LeafSizeVoxel, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudOut);
// ---------------------------- Filter ----- end



int featureNormal(0);
// --------------------------- Feature --- begin
void FeatureNormal(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudIn, const float RadiusSearch, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);
void drawArrows(pcl::visualization::PCLVisualizer Viewer, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudIn, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, const float EveryNumber, const float size, const float length);

int KeyPointSIFTok(1);
void KeyPointSIFT(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr out, 
const float minScale, const float minContrast,
const int nOctaves, const int nScalesPer);
// 最小尺度的标准差； 关键点检测阈值；
// 金字塔层数； 每层的尺度数目
namespace pcl{
	template<>
	struct SIFTKeypointFieldSelector<PointXYZ>{
		inline float
			operator () (const PointXYZ &p) const{
			    return p.z;
		    }
	};
}



// --------------------------- Feature ----- end
//







/*
inline void timeLog(int timeTemp,int timeTotal, pcl::console::TicToc time){
    timeTemp = time.toc();
    time.tic();
    timeTotal += timeTemp;
    cout << "Prapare for normal time: " << timeTemp <<  "ms." << endl;
    outfile << "Prapare for normal: " << timeTemp << "ms." << endl;
}

*/








//=========================
//=======================
//====================
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


    //  1.2 ---------- load initialization
    ifstream information("../Initialization.qr");
    string file_temp;
    vector <string> file_inf;
    while(getline(information, file_temp)){
        if( file_temp.find("/")==-1 ||  file_temp.find("/")>=2   ){
            //cout<< file_temp;
            file_inf.push_back(file_temp);
        }
    }//cout << endl;
    string FileName = file_inf[0];
    /*
    [0] bunny01
    [1] bunny32497
    [2] 18
    */
    //cout << FileName << endl;
    



    pcl::visualization::PCLVisualizer viewer1("Filter Viewer");
    viewer1.setBackgroundColor(255, 255, 255);// 255 *3 = white
    //viewer1.setBackgroundColor(0, 255, 0);// 0,255,0= green




    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOri(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrixyz(new pcl::PointCloud<pcl::PointXYZ>);
    timeTemp = time.toc();
    time.tic();
    timeTotal += timeTemp;
    cout << "Initialization time: " << timeTemp  << "ms." << endl;
    outfile << "Initialization time: " << timeTemp << "ms."  << endl;

    // 1.3----------loading pcd file
    pcl::io::loadPCDFile(FileName, *cloudOri);
    pcl::io::loadPCDFile(FileName, *cloudOrixyz);
    timeTemp = time.toc(); time.tic();
    timeTotal += timeTemp;
    cout << "Loading file time: " << timeTemp <<  "ms." << endl << "Point number is: " << cloudOri->size() << endl;
    outfile << "Loading file time: " << timeTemp << "ms." << endl << "Point number is: " << cloudOri->size() << endl;

    
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGBA> fildColor(cloudOri, "z");
    //viewer1.addPointCloud<pcl::PointXYZRGBA>(cloudOri, fildColor, "cloudOri_zqr");
    viewer1.addPointCloud(cloudOri, "cloudOri_zqr");
    viewer1.setPointCloudRenderingProperties( pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 1, "cloudOri_zqr" );
    
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);


    //====================================
    //=========  Filters =========== begin

    //============ voxel grid...
    if(voxelGrid ==1){
        float LeafSizeVoxel1 = 0.01;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudVoxel(new pcl::PointCloud<pcl::PointXYZRGBA>);
    FilterVoxelGrid(cloudOri, LeafSizeVoxel1, CloudVoxel);
    viewer1.updatePointCloud(CloudVoxel, "cloudOri_zqr");
    }

    //=========  Filters ============= end
    //====================================







    //====================================
    //=========  Feature =========== begin

    //============ normal ...
    if(featureNormal ==1){
        float r = 0.01;
        pcl::PointCloud<pcl::Normal>::Ptr cloudNormal(new pcl::PointCloud<pcl::Normal>);
    timeTemp = time.toc(); time.tic();
    timeTotal += timeTemp;
    cout << "Prepare for normals time: " << timeTemp <<  "ms." << endl;
    outfile << "Prepare for normals: " << timeTemp << "ms." << endl;


        FeatureNormal(cloudOri, r, cloudNormal);
    timeTemp = time.toc(); time.tic();
    timeTotal += timeTemp;
    cout << "Compute normals time: " << timeTemp <<  "ms." << endl;
    outfile << "Compute normals time : " << timeTemp << "ms." << endl;
       
        //addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal>(cloudOri, cloudNormal, 300, 0.02f, "CloudNormal_zqr");
        //  normal, every 5 points, scale, id, viewport

        drawArrows(viewer1, cloudOri,cloudNormal, 100, 5, 0.03);// every n number; angle; lengthSize
    timeTemp = time.toc(); time.tic();
    timeTotal += timeTemp;
    cout << "Drow normals time: " << timeTemp <<  "ms." << endl;
    outfile << "Drow normals time : " << timeTemp << "ms." << endl;


        // add color dead end...
        /*
        // the function doesn't support that...
        pcl::PointCloud<pcl::PointXYZ> cloudCurvature;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCurvature66(new pcl::PointCloud<pcl::PointXYZ>);
        cloudCurvature.width =cloudOri->size(); 
        cloudCurvature.height = 1;
        cloudCurvature.is_dense = false;
        cloudCurvature.points.resize (cloudCurvature.width * cloudCurvature.height);
        for (size_t i = 0; i < cloudCurvature.points.size (); ++i){
            cloudCurvature.points[i].x = cloudNormal->points[i].curvature;
            cloudCurvature.points[i].y = 0;
            cloudCurvature.points[i].z = 0;
        }
        pcl::copyPointCloud(cloudCurvature, *cloudCurvature66);
        
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloudCurvature66, "x");
        viewer1.addPointCloud<pcl::PointXYZ>(cloudOri, cloudCurvature66, "cloudOri_zqr");
        viewer1.setPointCloudRenderingProperties( pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, "cloudOri_zqr" );
        */
    }

    


    // 
    if(KeyPointSIFTok==1){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSIFT (new pcl::PointCloud<pcl::PointXYZ>);
    timeTemp = time.toc(); time.tic();
    timeTotal += timeTemp;
    cout << "SIFT prepare time: " << timeTemp <<  "ms." << endl;
    outfile << "SIFT prepare time : " << timeTemp << "ms." << endl;
        KeyPointSIFT(cloudOrixyz, cloudSIFT , 0.001f, 1.0f, 1, 3);
        // const float minScale, const float minContrast,const int nOctaves, const int nScalesPer
    timeTemp = time.toc(); time.tic();
    timeTotal += timeTemp;
    cout << "SIFT calculate time: " << timeTemp <<  "ms." << endl ;
    outfile << "SIFT calculate time : " << timeTemp << "ms." << endl;
        
        cout << "sift point number is: "<<cloudSIFT->size()<<endl;
        outfile << "sift point number is: "<<cloudSIFT->size()<<endl;
        viewer1.addPointCloud(cloudSIFT, "SIFTkeypoints");
        viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"SIFTkeypoints");
    }
    

    //=========  Feature ============= end
    //====================================
    








    //pcl::PolygonMesh mesh_try;
    //pcl::io::loadPLYFile(argv[1], mesh_try);//"output.ply"
    //viewer1.addPolygonMesh( mesh_try, "mesh_zqr");





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
//==========================================
//==========================================
//==========================================















//====================================
//=========  Filters =========== begin

void FilterVoxelGrid(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudIn, const float LeafSizeVoxel, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudOut){
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (CloudIn);
    sor.setLeafSize (LeafSizeVoxel,LeafSizeVoxel,LeafSizeVoxel);
    sor.filter (*CloudOut);
}

//=========  Filters ============= end
//====================================










//====================================
//=========  Feature =========== begin

void FeatureNormal(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudIn, const float RadiusSearch, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud(cloudIn);
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGBA>());
    ne.setSearchMethod(kdtree);
    ne.setRadiusSearch(RadiusSearch);//LIKE 0.03m
    ne.compute(*cloud_normals);
    cout << "Normal ok" << endl;
    cout << "(nx, ny, nz, curvature)= "<< cloud_normals->points[0].normal_x << ", "<< cloud_normals->points[0].normal_y << ", "<< cloud_normals->points[0].normal_z << ", "<< cloud_normals->points[0].curvature << "."<<endl;
}

//=========  Feature ============= end
//====================================


//
void drawArrows(pcl::visualization::PCLVisualizer viewer, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudIn, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, const float EveryNumber, const float angle, const float length){
    
    //pcl::visualization::PCLVisualizer Viewer2("Filter Viewer");
    //Viewer2.setBackgroundColor(255, 255, 255);// 255 *3 = white
    //Viewer2.addPointCloud(cloudIn, "cloudOri_zqr2");

    for(size_t i=0; i<cloudIn->size(); i=i+EveryNumber ){
        pcl::ModelCoefficients cone_z;
			cone_z.values={
                cloudIn->points[i].x-length*cloud_normals->points[i].normal_x ,
                cloudIn->points[i].y- length*cloud_normals->points[i].normal_y,
                 cloudIn->points[i].z-length*cloud_normals->points[i].normal_z,
				length*cloud_normals->points[i].normal_x,length*cloud_normals->points[i].normal_y,length*cloud_normals->points[i].normal_z,
                 
                 angle};
			viewer.addCone(cone_z,   "cone_z"+std::to_string(i));
			//viewer.addSphere( z_normal, 10.0,  255, 0, 0,  "sphere_z");
    }
    cout << "normals are drawn." << endl;
}



void KeyPointSIFT(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr out, 
const float minScale, const float minContrast,
const int nOctaves, const int nScalesPer){
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> siftkeys01;
    //cout << "step 01..." << endl;
    sift.setInputCloud(cloudIn);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    sift.setSearchMethod(tree);
    //cout << "step 02..." << endl;
    sift.setScales(minScale, nOctaves, nScalesPer);
    //cout << "step 03..." << endl;
    sift.setMinimumContrast(minContrast);
    //cout << "step 04..." << endl;
    sift.compute(siftkeys01);cout << "step 05..." << endl;
    copyPointCloud(siftkeys01, *out);
    
}

