/* ===========
//  * Author: Shepherd Qirong
//  * Date: 2020-04-28 18:58:11
//  * Github: https://github.com/ShepherdQR
//  * LastEditors: Shepherd Qirong
//  * LastEditTime: 2020-05-17 08:31:23
//  * Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
*/
/*
the code is copied as follows:
https://blog.csdn.net/y3254321/article/details/72157739
*/


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>

#include <time.h>

using namespace std;
 


int main(int argc, char** argv){

	time_t timeLocal99 = std::time(NULL);
    char timeCurrent99[64];
    strftime(timeCurrent99, sizeof(timeCurrent99), "%Y-%m-%d%H:%M:%S",localtime(&timeLocal99));
	string timeCurrent100 = timeCurrent99;


	// 确定文件格式
	char tmpStr[100];
	strcpy(tmpStr, argv[1]);
	char* pext = strrchr(tmpStr, '.');
	std::string extply("ply");
	std::string extpcd("pcd");
	if (pext){
		*pext = '\0';
		pext++;
	}
	std::string ext(pext);
	//如果不支持文件格式，退出程序
	if (!((ext == extply) || (ext == extpcd))){
		std::cout << "文件格式不支持!" << std::endl;
		std::cout << "支持文件格式：*.pcd和*.ply！" << std::endl;
		return(-1);
	}
 
	
	pcl::PointCloud < pcl::PointXYZRGB > ::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //创建点云对象指针，用于存储输入
	if (ext == extply){
		if (pcl::io::loadPLYFile(argv[1], *cloud) == -1){
			PCL_ERROR("Could not read ply file!\n");
			return -1;
		}
	}
	else{
		if (pcl::io::loadPCDFile(argv[1], *cloud) == -1){
			PCL_ERROR("Could not read pcd file!\n");
			return -1;
		}
	}

	pcl::visualization::PCLVisualizer viewer1("Ori");
    viewer1.setBackgroundColor(255, 255, 255);// 255 *3 = white
	viewer1.addPointCloud(cloud , "ori");
 
	// generate tree2 for searching, we do not use..
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(6);
	n.compute(*normals);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloud_with_normals);


	pcl::Poisson<pcl::PointXYZRGBNormal> pn;
	pn.setDegree(2); //设置参数degree[1,5] ??
	pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
	pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度

	pn.setConfidence(false);
	pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）

	//pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
	//pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
	//pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
	//pn.setIndices();
 
	//pn.setSearchMethod(tree2);
	pn.setInputCloud(cloud_with_normals);

	pcl::PolygonMesh mesh;
	pn.reconstruct(mesh);//or: pn.performReconstruction(mesh);
 
	//保存网格图
	pcl::io::savePLYFile("Bunnyresult" + timeCurrent100 +".ply", mesh);
 
	// 显示结果图
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPolygonMesh(mesh, "my");
	viewer->addCoordinateSystem();
	viewer->initCameraParameters();
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);
		viewer1.spinOnce();
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
 
	return 0;
}