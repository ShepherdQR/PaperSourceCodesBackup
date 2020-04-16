/* ===========
//  * Author: Shepherd Qirong
//  * Date: 2020-04-13 23:52:28
//  * Github: https://github.com/ShepherdQR
//  * LastEditors: Shepherd Qirong
//  * LastEditTime: 2020-04-17 04:06:26
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

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>


#include <fstream>
#include <iostream>
#include <string>

#include <vector>

using namespace cv;
using namespace std;


void index2Color(Mat imageFill, int ii, int jj, int colorLabelP);
int colorLabelF(Vec3b intensity);



string picsFolder = "/home/jellyfish/cpps/FinalPaperSourceCodes/Pics/";
string PicFileName = "LeftBottomRGBY03";


string fileName1 = "/home/jellyfish/cpps/FinalPaperSourceCodes/s1StructurePointCloudSimulation-qr0410";
string scanFiles = fileName1 +"/ScanClouds/";

string whichPcd2Load = "/Vox-trans-00SL.pcd";


int main( int argc, char** argv ){


    ifstream information("/home/jellyfish/cpps/FinalPaperSourceCodes/AutoScanProject-20200324/commonSettings/Settings.log");
    string file_temp;
    vector <string> file_inf;
    while(getline(information, file_temp)){
        if( file_temp.find("/")==-1 ||  file_temp.find("/")>=2   ){
            //cout<< file_temp;
            file_inf.push_back(file_temp);
        }
    }//cout << endl;
    //string FileFolderName = scanFiles + file_inf[0];
    string FileFolderName = scanFiles + "2020-04-1501:31:49";
    

    pcl::visualization::PCLVisualizer viewer1("Ori"), viewer2("Vox");
    viewer1.setBackgroundColor(255, 255, 255);// 255 *3 = white
    viewer2.setBackgroundColor(255, 255, 255);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAllOri(new pcl::PointCloud<pcl::PointXYZ>),cloudAllVox(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAllOritrans(new pcl::PointCloud<pcl::PointXYZ>),cloudAllVoxtrans(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCur(new pcl::PointCloud<pcl::PointXYZ>);
    string loadPcdFIle = FileFolderName + whichPcd2Load;
    pcl::io::loadPCDFile(loadPcdFIle, *cloudCur);
    cout << loadPcdFIle << endl;

    viewer1.addPointCloud(cloudCur , "try");

    viewer1.setPointCloudRenderingProperties( pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, "try");

    viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,1, "try");
    






    string imageName( picsFolder + PicFileName +  ".png" );
    Mat iIn= imread( imageName ); 
    //default = IMREAD_COLORIM.   READ_GRAYSCALE
    //CV_UNUSED(iIn);

    namedWindow( "Display window", WINDOW_AUTOSIZE ); 
    imshow( "Display window", iIn );  

    int nRows = iIn.rows;
    int nCols = iIn.cols;
    cout << nRows << endl;


    Mat imageFill= iIn.clone();
    //Mat imageFillCopy= imageFill.clone();

    for(size_t  i = 0; i < nRows; ++i){
        for(size_t j = 0; j < nCols; ++j){
            Vec3b intensity = imageFill.at<Vec3b>(i, j);
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
                        Vec3b intensityP1 = imageFill.at<Vec3b>((int)(i-indi), j);
                        int colorLabelP1 = colorLabelF( intensityP1 );
                        if( colorLabelP1 != 4 ){
                            whichColor[0][0]= colorLabelP1;
                            whichColor[0][1]= -1 * indi;
                            s11 =1;
                        } 
                    }
                    if((i+indi) < nRows){
                        Vec3b intensityP2 = imageFill.at<Vec3b>((i+indi), j);
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

                index2Color(imageFill, i, j, colorLabelP);
                // imageFill.at<Vec3b>(i, j).val[0]=255;
                // imageFill.at<Vec3b>(i, j).val[1]=255;
                // imageFill.at<Vec3b>(i, j).val[2]=255;
            }
        }
    }
    imshow("imageFill",  imageFill);
    imwrite(picsFolder + PicFileName +  "Narrow.png", imageFill);


    // Mat iOut;
    // int rWidth = 80, rHeight = 12;
    // cv::resize(iIn, iOut, cv::Size(rWidth, rHeight), INTER_AREA);//INTER_LINEAR
    // imshow("imageOut", iOut);

    

    
    //waitKey();
    viewer1.addCoordinateSystem();  
    while ((!viewer1.wasStopped()) & (!viewer2.wasStopped()) ){
		viewer1.spinOnce();
        viewer2.spinOnce();
        
	}
    


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