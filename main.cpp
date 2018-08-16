#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include "utils.h"
#include "segmentation.h"
#include "feature_circle.h"
#include "contours.h"
#include "dfs.h"
using namespace cv;
using namespace std;

//prior: radius, n*n the num in each rectangle(error<3)
// the number(threshold) of circles that can be segmented(0.9)  
//可乐21, 24*9 4*6 农夫11.7 28*10pixel 4*7 315*163.3
double assume_radius = 21;
int m =6;//x
int n =4;//y 
float height = 297.8;
float  width= 194.4;
int open_scale = round(assume_radius*0.25);
int close_scale = round(assume_radius*2);
int poly_approx_scale = round(1.2*assume_radius);







int main(int argc, char **argv) {
    

 for(int i = 4;i<=24;i++){
     /* cv::String p1 = "kuoluo/"+to_string(i)+"_texture.bmp";
      cv::String p2 = "kuoluo_60/"+to_string(i)+"_texture.bmp";
      cv::String p3 = "kuoluo/"+to_string(i)+".bmp";
      cv::String p4 = "kuoluo_60/"+to_string(i)+".bmp";
      float angle = 60;
      if(angle == 90){
          Mat in_img1 = imread(p1);
          Mat in_img2 = imread(p3);
          Mat out_img1,out_img2;
          rotate(in_img1,out_img1,cv::RotateFlags::ROTATE_90_CLOCKWISE);
          rotate(in_img2,out_img2,cv::RotateFlags::ROTATE_90_CLOCKWISE);
          cv::imwrite(p2,out_img1);
          cv::imwrite(p4,out_img2);
          continue;
    }
      rotateImg(p1,p2,angle);
      rotateImg(p3,p4,angle);*/
      
      
      
    //int i = 14 ;
    Mat srcImage = imread("kuoluo/"+to_string(i)+"_texture.bmp",0);
    Mat mask = imread("kuoluo/"+to_string(i)+".bmp",0);
    threshold(mask,mask,100,255,THRESH_BINARY);
    Mat roi;
    srcImage.copyTo(roi,mask);
    //imshowResize("the "+to_string(i)+" img",mask);
    //waitKey(0);
    
    vector<int> params;
    params.push_back(round(width));
    params.push_back(round(height));  
    params.push_back(m);
    params.push_back(n);
    params.push_back(round(assume_radius));
    params.push_back(open_scale);
    params.push_back(close_scale);
    params.push_back(poly_approx_scale);
    
   
    int t = clock();
    segmentation_roi(roi,mask,FEATURE_USE_CIRCLE,params);
    int t2 = clock();
   cout <<"time "<< (t2-t)/CLOCKS_PER_SEC<<endl;
   cout <<"this is the "<<i<<" img"<<endl;
    
    }
   
    return 0;
}





