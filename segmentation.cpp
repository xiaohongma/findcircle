
#include "segmentation.h"
#include "utils.h"
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include "dfs.h"
#include "feature_circle.h"
#include "utils.h"
using namespace cv;
using namespace std;
int segmentation_roi(cv::Mat& roi, cv::Mat& mask, int feature, std::vector< int >& param)
{
    
    if(param.size()<8 ){
        cout <<"too few arguments!"<<endl;
        return 0;
    }
    vector<StepInfo> path;
    vector<PathInfo> allpaths;
    int width = param.at(0);
    int height =  param.at(1);
    // actually we only use the first two direction.
    int direction[8][2] = {{width,height},{height,width},{-width,height},{width,-height},{-width,-height},{-height,width},{height,-width},{-height,-width}};
    Mat visited = ~mask;// visited map: used in dfs. It is reverse of mask
 
    // we can use anyone of the following three features in dfs.However, we just implement the circle feature now. 
    switch(feature){
        case FEATURE_USE_CIRCLE:
        {
            vector<Point> centers;
            detection_circles(roi,param,centers);
            dfs(visited,roi,mask,direction,feature,centers,param, path, allpaths);
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
            cout <<"please choose a right feature"<<endl;
    }

    // show segmentation results
    printoutPath(roi,allpaths,false);
    
   
    return 0;
}

void printoutPath(Mat& img,vector<PathInfo>& allpath,bool only_show_best_path)
{
    cout <<allpath.size()<<"paths have been found" << endl;
    
    if(only_show_best_path){
         PathInfo bestpath;
        if(allpath.empty()){
            cout <<"no segmentation path have been found"<<endl;
            return;
        }
        float score = get_mean_score(allpath.at(0).path);
        int index = 0;
        if(allpath.size()>1){
            for(int i = 1;i<allpath. size();i++){
            //vector<Step> path = allpath.at(i).path; 
            float s = get_mean_score(allpath.at(i).path);
            cout<<"score"<<s<<endl;
                if(s>score) {
                    index = i;
                    score = s;
                }
            } 
        }

        cout <<allpath.size()<<"paths have been found,the the best path is "<<index<< endl;
        bestpath = allpath.at(index);
        
        for(int j = 0;j<bestpath.path.size();j++){
            DrawRotatedRect(img,bestpath.path.at(j).r_rect,Scalar(255));
             putText(img,to_string(j+1),bestpath.path.at(j).r_rect.center,FONT_HERSHEY_SCRIPT_SIMPLEX,4,Scalar(255),4);
            cout<<"path "<<bestpath.path.at(j).r_rect.center<<" direction "<<bestpath.path.at(j).direction<<" score "<<bestpath.path.at(j).score<<endl; 
        }
        putText(img,"score "+floatToString(score,2),Point(100,100),FONT_HERSHEY_SIMPLEX,2,Scalar(255),4);
        imshowResize("best segmentation result",img);
        waitKey(0);
        
    }else{//printout all paths
        for(int m = 0;m<allpath.size();m++){
            PathInfo path = allpath.at(m);
            float score = get_mean_score(path.path);
            Mat img_c = img.clone();
           for(int j = 0;j<path.path.size();j++){
                DrawRotatedRect(img_c,path.path.at(j).r_rect,Scalar(255));
                putText(img_c,to_string(j+1),path.path.at(j).r_rect.center,FONT_HERSHEY_SCRIPT_SIMPLEX,4,Scalar(255),4);
                cout<<"path "<<j<<" "<<path.path.at(j).r_rect.center<<" direction "<<path.path.at(j).direction<<" score "<<path.path.at(j).score<<endl; 
            }
            putText(img_c,"score "+floatToString(score,2),Point(100,100),FONT_HERSHEY_SIMPLEX,2,Scalar(255),4);
            imshowResize("segmentation result path "+to_string(m),img_c);
            waitKey(0);
           
        }
    }
    
    
        
}


float get_mean_score(vector<StepInfo>& path){
    if(path.empty()) return 0;
    int num = path.size();
    
    float acc = 0;
    for(int i = 0; i<num;i++){
     acc += path.at(i).score;   
    }
    return acc/num;
}


