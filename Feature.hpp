
#ifndef FEATURE_HPP
#define FEATURE_HPP
#include <opencv2/opencv.hpp>
#include "utils.h"
#include "dfs.hpp"
#include "polygon_calculation.hpp"
class Feature{
   /* private:
        cv::Mat visited;
        cv::Mat img;
        cv::Mat mask;
        std::vector<int> params;
        std::vector<cv::Vec2i> direction;
        std::vector<cv::Point> centers;*/
public:
    Feature(){
        
    }
     /*Feature(cv::Mat& img,cv::Mat& mask,std::vector<int>& params){
     this->img = img;
     this->mask = mask;
     this->params = params;
    
    }*/
    
public:
    virtual Feature* GetClassType(void){
        return this;
    }
    
};
    
#endif
