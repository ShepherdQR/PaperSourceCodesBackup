/* ===========
//  * Author: Shepherd Qirong
//  * Date: 2020-05-29 23:46:36
//  * Github: https://github.com/ShepherdQR
//  * LastEditors: Shepherd Qirong
//  * LastEditTime: 2020-05-30 03:27:53
//  * Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
*/
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/gasd.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>

//#include <boost/thread/thread.hpp>



#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>


#include <pcl/visualization/histogram_visualizer.h> //直方图的可视化
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/console/time.h>   // TicToc

#include <string>
using namespace std;


int pcdNumber = 4;
string pcdFolderName[] = {"vox-Tank1-T99A","vox-Tank2-2A5","vox-Tank3-T90A"},
pcdFolderNameL = "/home/jellyfish/cpps/FinalPaperSourceCodes/s3Registration-qr0524/ImportantResults/vox-0_04-20200525-03:23:48/",
pcdFolderNameR = "-20200525-03:23:48.pcd";

string file4Pcd = "/home/jellyfish/cpps/FinalPaperSourceCodes/s3Registration-qr0524/voxelPcds/vox-0_02-20200525-06:52:14/vox-Tank1-T99A-20200525-06:52:14.pcd";

/*
0
Point size: 19720
Calculate pcd 0 caused time: 432 ms.//0.219ms/p
fphf feature size : 19720
hi1
2.41293, 0.810074, 0.49336, 0.902992, 6.1967, 81.2191, 4.02463, 0.533122, 0.390858, 0.779757, 2.20075, 0.961019, 0.711077, 1.38118, 3.36705, 9.35785, 68.5463, 9.40705, 3.42598, 1.36181, 0.734, 0.711263, 1.6212, 1.51071, 1.924, 4.59012, 7.81555, 57.4196, 14.7241, 5.88417, 2.61523, 1.08192, 0.777988, 
Calculate Rest pcd 0 caused time: 105 ms.


1
Point size: 13051
Calculate pcd 1 caused time: 299 ms.//0.229ms/p
fphf feature size : 13051
hi1
2.13842, 0.893415, 0.697187, 2.38463, 11.264, 71.8512, 6.04121, 1.49782, 0.479404, 1.11621, 1.59821, 2.12638, 1.63885, 2.78161, 4.90744, 9.4754, 59.2822, 9.36246, 4.91958, 2.69112, 1.68237, 1.09428, 1.5036, 2.79704, 3.21402, 5.3998, 8.06209, 47.5878, 12.1274, 9.89769, 5.05249, 3.21686, 1.10295, 
Calculate Rest pcd 1 caused time: 71 ms.


2
Point size: 19053
Calculate pcd 2 caused time: 415 ms.// 0.0217
fphf feature size : 19053
hi1
2.5691, 0.892877, 0.526745, 1.23817, 7.45874, 77.7995, 5.10432, 0.899187, 0.42441, 0.915719, 2.04003, 1.67392, 1.15406, 1.82765, 3.76305, 9.96454, 63.9702, 9.99834, 3.6995, 1.80715, 1.10362, 0.906914, 1.77743, 1.86467, 2.45238, 5.17264, 10.9883, 50.4505, 13.7606, 6.5358, 3.52038, 1.89377, 1.45216, 
Calculate Rest pcd 2 caused time: 103 ms.


3
Point size: 102104
Calculate pcd 3 caused time: 5629 ms.===0.055ms/p
fphf feature size : 102104
hi1
1.79757, 0.536151, 0.343601, 0.652293, 5.46991, 85.1864, 3.14013, 0.357142, 0.250631, 0.486442, 1.77805, 0.386942, 0.412997, 0.88296, 2.69627, 8.67221, 73.957, 8.65339, 2.69825, 0.881948, 0.420224, 0.335272, 1.13519, 1.14234, 1.59353, 4.41183, 6.85167, 61.4466, 13.9139, 5.95846, 2.26996, 0.811922, 0.462317, 
Calculate Rest pcd 3 caused time: 530 ms.


33维度的欧式距离：
Score i is: 8.30342
Score i is: 26.4106
Score i is: 17.8468

*/

bool step[3]={false, true, false};

int main( int argc, char** argv ){

    pcl::console::TicToc time;
    int timeTotal(0), timeRest(0);

    Eigen::Matrix<float, 33, 4> featureMatrix;
    featureMatrix.fill(0);

if(step[1]){
    float a[]={2.41293, 0.810074, 0.49336, 0.902992, 6.1967, 81.2191, 4.02463, 0.533122, 0.390858, 0.779757, 2.20075, 0.961019, 0.711077, 1.38118, 3.36705, 9.35785, 68.5463, 9.40705, 3.42598, 1.36181, 0.734, 0.711263, 1.6212, 1.51071, 1.924, 4.59012, 7.81555, 57.4196, 14.7241, 5.88417, 2.61523, 1.08192, 0.777988},
    b[] = {2.13842, 0.893415, 0.697187, 2.38463, 11.264, 71.8512, 6.04121, 1.49782, 0.479404, 1.11621, 1.59821, 2.12638, 1.63885, 2.78161, 4.90744, 9.4754, 59.2822, 9.36246, 4.91958, 2.69112, 1.68237, 1.09428, 1.5036, 2.79704, 3.21402, 5.3998, 8.06209, 47.5878, 12.1274, 9.89769, 5.05249, 3.21686, 1.10295},
    c[] = {2.5691, 0.892877, 0.526745, 1.23817, 7.45874, 77.7995, 5.10432, 0.899187, 0.42441, 0.915719, 2.04003, 1.67392, 1.15406, 1.82765, 3.76305, 9.96454, 63.9702, 9.99834, 3.6995, 1.80715, 1.10362, 0.906914, 1.77743, 1.86467, 2.45238, 5.17264, 10.9883, 50.4505, 13.7606, 6.5358, 3.52038, 1.89377, 1.45216},
    d[] = {1.79757, 0.536151, 0.343601, 0.652293, 5.46991, 85.1864, 3.14013, 0.357142, 0.250631, 0.486442, 1.77805, 0.386942, 0.412997, 0.88296, 2.69627, 8.67221, 73.957, 8.65339, 2.69825, 0.881948, 0.420224, 0.335272, 1.13519, 1.14234, 1.59353, 4.41183, 6.85167, 61.4466, 13.9139, 5.95846, 2.26996, 0.811922, 0.462317 };

    Eigen::Matrix<float, 33, 1> featurePTar;
    featurePTar  << 1.79757, 0.536151, 0.343601, 0.652293, 5.46991, 85.1864, 3.14013, 0.357142, 0.250631, 0.486442, 1.77805, 0.386942, 0.412997, 0.88296, 2.69627, 8.67221, 73.957, 8.65339, 2.69825, 0.881948, 0.420224, 0.335272, 1.13519, 1.14234, 1.59353, 4.41183, 6.85167, 61.4466, 13.9139, 5.95846, 2.26996, 0.811922, 0.462317;
    Eigen::Matrix<float, 33, 1> featurePCur;

    //Eigen::MatrixXf::Map(d, 33, 1) += featurePTar;

    float distance[pcdNumber-1]={0};
    for(int i=0; i<pcdNumber-1; ++i){
        if(i==0) featurePCur<< 2.41293, 0.810074, 0.49336, 0.902992, 6.1967, 81.2191, 4.02463, 0.533122, 0.390858, 0.779757, 2.20075, 0.961019, 0.711077, 1.38118, 3.36705, 9.35785, 68.5463, 9.40705, 3.42598, 1.36181, 0.734, 0.711263, 1.6212, 1.51071, 1.924, 4.59012, 7.81555, 57.4196, 14.7241, 5.88417, 2.61523, 1.08192, 0.777988;
        if(i==1) featurePCur<< 2.13842, 0.893415, 0.697187, 2.38463, 11.264, 71.8512, 6.04121, 1.49782, 0.479404, 1.11621, 1.59821, 2.12638, 1.63885, 2.78161, 4.90744, 9.4754, 59.2822, 9.36246, 4.91958, 2.69112, 1.68237, 1.09428, 1.5036, 2.79704, 3.21402, 5.3998, 8.06209, 47.5878, 12.1274, 9.89769, 5.05249, 3.21686, 1.10295;
        if(i==2) featurePCur<< 2.5691, 0.892877, 0.526745, 1.23817, 7.45874, 77.7995, 5.10432, 0.899187, 0.42441, 0.915719, 2.04003, 1.67392, 1.15406, 1.82765, 3.76305, 9.96454, 63.9702, 9.99834, 3.6995, 1.80715, 1.10362, 0.906914, 1.77743, 1.86467, 2.45238, 5.17264, 10.9883, 50.4505, 13.7606, 6.5358, 3.52038, 1.89377, 1.45216;
        featurePCur = featurePCur - featurePTar;
        distance[i] = featurePCur.norm();
    }
    for(int i=0; i< pcdNumber-1; ++i){
        cout << "Score i is: " << distance[i] << endl;
    }




}






if(step[0]){   
    for(size_t i=3; i<4; ++i){
        time.tic();
        cout << i << endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        
        string curPcdPath;
        if(i==pcdNumber-1){
            curPcdPath = file4Pcd;
        }else
        curPcdPath = pcdFolderNameL + pcdFolderName[i] + pcdFolderNameR;

        pcl::io::loadPCDFile(curPcdPath, *cloud);

        cout << "Point size: " << cloud->points.size() <<   endl;

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch (0.08);
        ne.compute (*cloud_normals);


        pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh;

        fpfh.setInputCloud (cloud);
        fpfh.setInputNormals (cloud_normals);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
        fpfh.setSearchMethod (tree2);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_fe_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
        fpfh.setRadiusSearch (0.12);
        fpfh.compute (*fpfh_fe_ptr);

        int timeI = time.toc();timeTotal +=timeI;
        cout << "Calculate pcd " << i << " caused time: " << timeI << " ms." << endl;
        time.tic();

        cout << "fphf feature size : " << fpfh_fe_ptr->points.size() << endl;

        for(int j=0;j< (fpfh_fe_ptr->points.size())  ;++j){
            for(int k=0;k<33;++k){
                featureMatrix(k,i) =featureMatrix(k,i) + fpfh_fe_ptr->points[j].histogram[k]  ;
            }
        }
        cout << "hi1" << endl;
        for(int k=0;k<33;++k){
            featureMatrix(k,i) = featureMatrix(k,i) / (fpfh_fe_ptr->points.size() ) ;
            cout << featureMatrix(k,i) << ", ";
        }
        cout  << endl;

        timeRest += time.toc();
        cout << "Calculate Rest pcd " << i << " caused time: " << timeRest << " ms." << endl;
    }






    time.tic();
    for(int i=0; i< 4; ++i){
        
        for(int j=0; j< 33; ++j){
            cout << featureMatrix(j,i) << ", ";
        }
        cout << endl;
    }

    Eigen::Matrix<float, 33, 1> featurePTar, featurePCur;
    featurePTar.fill(0);featurePCur.fill(0);
    featurePTar = featureMatrix.col(pcdNumber-1);

    float distance[pcdNumber-1]={0};
    for(int i=0; i<pcdNumber-1; ++i){
        featurePCur = featureMatrix.col(i) - featurePTar;
        distance[i] = featurePCur.norm();
    }
    for(int i=0; i< pcdNumber-1; ++i){
        cout << "Score i is: " << distance[i] << endl;
    }

}


    timeRest += time.toc();
    cout << "total time is: " << timeTotal + timeRest << " ms."<< endl;




        // pcl::visualization::PCLHistogramVisualizer view;
        // view.setBackgroundColor(0,0,0);
        // view.addFeatureHistogram<pcl::FPFHSignature33> (*fpfh_fe_ptr,"fpfh",100); //对下标为100的点的直方图特征可视化
        // view.spinOnce();
        // pcl::visualization::PCLPlotter plotter;
        // plotter.addFeatureHistogram(*fpfh_fe_ptr, 300); //设置的很坐标长度，该值越大，则显示的越细致
        // plotter.plot();

    

















        // pcl::GASDEstimation<pcl::PointXYZ, pcl::GASDSignature512> gasd;
        // gasd.setInputCloud (cloud);
        // pcl::PointCloud<pcl::GASDSignature512> descriptor;
        // cout << size_t( descriptor[0].descriptorSize ()) << endl;
        // gasd.compute (descriptor);
        // //Eigen::Matrix4f trans = gasd.getTransform ();
        // cout << "GASD descriptor[0] of " << pcdFolderName[i] << " is: " << endl;
        // for (size_t j = 0; j < size_t( descriptor[0].descriptorSize ()); ++j){
        //     cout << descriptor[0].histogram[j] << endl;
        // }
    


    return 0;
}