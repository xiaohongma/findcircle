#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include "utils.h"
#include "segmentation.hpp"
#include "dfs.hpp"
//prior: radius, n*n the num in each rectangle(error<3)
// the number(threshold) of circles that can be segmented(0.9)  
//可乐21, 24*9 4*6 农夫11.7 28*10pixel 4*7 315*163.3
double d_assume_radius = 21;
int n_m =6;//x
int n_n =4;//y 
float f_height = 297.8;
float  f_width= 194.4;
int n_open_scale = round(d_assume_radius*0.25);
int n_close_scale = round(d_assume_radius*2);
int n_poly_approx_scale = round(1.2*d_assume_radius);







int main(int argc, char **argv) {
    

 for(int i = 4;i<=24;i++){
    //int i = 4 ;
    cv::Mat mat_src_image = cv::imread("kuoluo/"+std::to_string(i)+"_texture.bmp",0);
    cv::Mat mat_mask = cv::imread("kuoluo/"+std::to_string(i)+".bmp",0);
    threshold(mat_mask,mat_mask,100,255,cv::THRESH_BINARY);
    cv::Mat mat_roi;
    mat_src_image.copyTo(mat_roi,mat_mask);
    //imshowResize("the "+to_string(i)+" img",mask);
    //waitKey(0);
    std::vector<int> vec_params;
    vec_params.push_back(round(f_width));
    vec_params.push_back(round(f_height));  
    vec_params.push_back(n_m);
    vec_params.push_back(n_n);
    vec_params.push_back(round(d_assume_radius));
    vec_params.push_back(n_open_scale);
    vec_params.push_back(n_close_scale);
    vec_params.push_back(n_poly_approx_scale);
    
     std::vector<PathInfo> vec_path_allpaths;
    Segmentation m_segmentation(mat_roi,mat_mask,vec_params);
    int time_t = clock();
    m_segmentation.segment_roi(FEATURE::FEATURE_USE_CIRCLE,vec_path_allpaths);
    // show segmentation results
    m_segmentation.printoutPath(vec_path_allpaths,false);
    int time_t2 = clock();
   std::cout <<"time "<< (time_t2-time_t)/CLOCKS_PER_SEC<<std::endl;
   std::cout <<"this is the "<<i<<" img"<<std::endl;
    }
    
    
    
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
   
    return 0;
}





