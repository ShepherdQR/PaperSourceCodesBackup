/* ===========
//  * Author: Shepherd Qirong
//  * Date: 2020-04-13 23:52:28
//  * Github: https://github.com/ShepherdQR
//  * LastEditors: Shepherd Qirong
//  * LastEditTime: 2020-04-15 03:53:01
//  * Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
*/
//  2020-04-13 23:53:31
/*
    Seperate this part
*/

//  2020-04-15 01:50:29
/*
    Yesterday I drop the idea to do rgs in opencv, cause the process is more easier in the preprocess of the pattern pictures.
    And for the display, I can do it using opengl or other lib, but to simply the process, I just add cubes with pcl.
*/
//  2020-04-15 03:52:29
/*
    I have a better way. And I'll fix it tommorrow.
*/


#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>

#include <iostream>
#include <string>

using namespace cv;
using namespace std;



int main( int argc, char** argv ){
    string imageName( "/home/jellyfish/cpps/FinalPaperSourceCodes/Pics/RightBottomRGBY01.png" );

    Mat iIn= imread( imageName ); 
    //default = IMREAD_COLORIM.   READ_GRAYSCALE
    //CV_UNUSED(iIn);


    namedWindow( "Display window", WINDOW_AUTOSIZE ); 
    imshow( "Display window", iIn );  

    int nRows = iIn.rows;
    int nCols = iIn.cols;

    for(size_t i = 0; i < nRows; ++i){
        Vec3b intensity = iIn.at<Vec3b>(i, 0);
        float blue = intensity.val[0];
        float green = intensity.val[1];
        float red = intensity.val[2];
        if( red != 255){
            cout << blue  << ",," << green << ",," << red <<endl;
        }
       
    }

    int neighborList[8][2] = {{0,-1},{1,-1}, {1,0}, {1,1}, {0,1}, {-1,-1}, {-1,0}, {-1,-1}};

    Mat imageFill;
    imageFill= iIn.clone();
    for(size_t i = 0; i < nRows; ++i){
        for(size_t j = 0; j < nCols; ++j){

            Vec3b intensity1 = imageFill.at<Vec3b>(i, j);
            float blue = intensity1.val[0];
            float green = intensity1.val[1];
            float red = intensity1.val[2];
            if((blue == 0) & (green == 0) & (red == 0)){
                int colorFLag[4]={0};//RGBY
                for(size_t k = 0; k < 8; ++k){
                    if((i+neighborList[k][0] >=0 ) & (i+neighborList[k][0] <=nCols ) & (j+neighborList[k][1] >=0 ) & (j+neighborList[k][1] <=nRows ) ){
                        Vec3b pNeighbor = imageFill.at<Vec3b>(i+neighborList[k][0], j+neighborList[k][1]);
                        float blue = pNeighbor.val[0];
                        float green = pNeighbor.val[1];
                        float red = pNeighbor.val[2];
                        if(blue==255){
                            colorFLag[2] +=1;
                            continue;
                        }if((red==255) & (green==0)){
                            colorFLag[0] +=1;
                            continue;
                        }if((red==0) & (green==255)){
                            colorFLag[1] +=1;
                            continue;
                        }if((red==255) & (green==255)){
                            colorFLag[3] +=1;
                            continue;
                        }
                    }
                }
                int colorLabel(0);
                for(size_t ii=1; ii<4; ++ii){
                    if( colorFLag[colorLabel]<colorFLag[ii] ){
                        colorLabel = ii;
                    }
                }
                if(colorLabel==0){
                    imageFill.at<Vec3b>(i, j).val[0]=0;
                    imageFill.at<Vec3b>(i, j).val[1]=0;
                    imageFill.at<Vec3b>(i, j).val[2]=255;
                }if(colorLabel==1){
                    imageFill.at<Vec3b>(i, j).val[0]=0;
                    imageFill.at<Vec3b>(i, j).val[1]=255;
                    imageFill.at<Vec3b>(i, j).val[2]=0;
                }if(colorLabel==2){
                    imageFill.at<Vec3b>(i, j).val[0]=255;
                    imageFill.at<Vec3b>(i, j).val[1]=0;
                    imageFill.at<Vec3b>(i, j).val[2]=0;
                }if(colorLabel==3){
                    imageFill.at<Vec3b>(i, j).val[0]=255;
                    imageFill.at<Vec3b>(i, j).val[1]=255;
                    imageFill.at<Vec3b>(i, j).val[2]=0;
                }
            }        
        }
    }
    imshow("imageFill",  imageFill);









    Mat iOut;

    int rWidth = 80, rHeight = 12;
    
    cv::resize(iIn, iOut, cv::Size(rWidth, rHeight), INTER_AREA);//INTER_LINEAR
    imshow("imageOut", iOut);

    waitKey(0);
    return 0;


}

//addCube (float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "cube", int viewport = 0);