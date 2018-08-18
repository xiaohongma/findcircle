#ifndef SEGMENTATION_HPP
#define SEGMENTATION_HPP
#include "utils.h"
#include "dfs.hpp"

class Segmentation{
private:
    cv::Mat img;
    cv::Mat mask;
    std::vector<int> params;

public:
    Segmentation(cv::Mat& img,cv::Mat& mask,std::vector<int>& params){
     this->img = img;
     this->mask = mask;
     this->params = params;
    }

    
  /**
* @brief this function is used to divide the roi into several box according to the feature
* 
* @param roi p_roi: input image
* @param feature indicates whether or not we use each feature, it is stored in fixed order:circle,contour,texture
* @param param stores prior params we have known. it is also in fixed order.first two is the width and height of the box, the third and forth are the numbers of cans along each side. And the fifth is stored radius. 
*/
int segment_roi(int feature,std::vector<PathInfo>& allpaths){
    
    if(params.size()<8 ){
        std::cout <<"too few arguments!"<<std::endl;
        return 0;
    }
 
    // we can use anyone of the following three features in dfs.However, we just implement the circle feature now. 
    switch(feature){
        case FEATURE_USE_CIRCLE:
        {
            //std::vector<cv::Point> centers;
            //detection_circles(img,param,centers);
            CircleFeature cf(img,mask,params);//initial
            DfsForDeconstruction Dd(img,mask,params);
            Dd.dfs(cf,allpaths);
            break;
            
        }
        case FEATURE_USE_CONTOUR:
            //vector<contour_feature> contours;
            //dtection_contour();
            //dfs(contours);
            break;
        case FEATURE_USE_TEXTURE:
           // detection_texture();
           // dfs(texture);
            break;
        default:
            std::cout <<"please choose a right feature"<<std::endl;
    }

    
   
    return 0;
}

/**
* @brief this function is used to printout the best segmentation path
* 
* @param allpaths storage all segmentation paths
*/
void printoutPath(std::vector< PathInfo >& allpath, bool only_show_best_path = true){
    std::cout <<allpath.size()<<"paths have been found" << std::endl;
    
    if(only_show_best_path){
         PathInfo bestpath;
        if(allpath.empty()){
            std::cout <<"no segmentation path have been found"<<std::endl;
            return;
        }
        float score = get_mean_score(allpath.at(0).path);
        int index = 0;
        if(allpath.size()>1){
            for(int i = 1;i<allpath. size();i++){
            //vector<Step> path = allpath.at(i).path; 
            float s = get_mean_score(allpath.at(i).path);
            std::cout<<"score"<<s<<std::endl;
                if(s>score) {
                    index = i;
                    score = s;
                }
            } 
        }

        std::cout <<allpath.size()<<"paths have been found,the the best path is "<<index<< std::endl;
        bestpath = allpath.at(index);
        
        for(int j = 0;j<bestpath.path.size();j++){
            DrawRotatedRect(img,bestpath.path.at(j).r_rect,cv::Scalar(255));
             putText(img,std::to_string(j+1),bestpath.path.at(j).r_rect.center,cv::HersheyFonts::FONT_HERSHEY_SCRIPT_SIMPLEX,4,cv::Scalar(255),4);
            std::cout<<"path "<<bestpath.path.at(j).r_rect.center<<" direction "<<bestpath.path.at(j).direction<<" score "<<bestpath.path.at(j).score<<std::endl; 
        }
        putText(img,"score "+floatToString(score,2),cv::Point(100,100),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(255),4);
        imshowResize("best segmentation result",img);
        cv::waitKey(0);
        
    }else{//printout all paths
        for(int m = 0;m<allpath.size();m++){
            PathInfo path = allpath.at(m);
            float score = get_mean_score(path.path);
            cv::Mat img_c = img.clone();
           for(int j = 0;j<path.path.size();j++){
                DrawRotatedRect(img_c,path.path.at(j).r_rect,cv::Scalar(255));
                putText(img_c,std::to_string(j+1),path.path.at(j).r_rect.center,cv::HersheyFonts::FONT_HERSHEY_SCRIPT_SIMPLEX,4,cv::Scalar(255),4);
                std::cout<<"path "<<j<<" "<<path.path.at(j).r_rect.center<<" direction "<<path.path.at(j).direction<<" score "<<path.path.at(j).score<<std::endl; 
            }
            putText(img_c,"score "+floatToString(score,2),cv::Point(100,100),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(255),4);
            imshowResize("segmentation result path "+std::to_string(m),img_c);
            cv::waitKey(0);
           
        }
    }
    
    
        
}

public:

/**
* @brief get the mean score of a path
* 
* @param path p_path:... every step
* @return float: mean score of every step
*/
float get_mean_score(std::vector<StepInfo>& path){
    if(path.empty()) return 0;
    int num = path.size();
    
    float acc = 0;
    for(int i = 0; i<num;i++){
     acc += path.at(i).score;   
    }
    return acc/num;
}
    
};

#endif
