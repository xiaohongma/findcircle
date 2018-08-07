
#include "segmentation.h"
#include "utils.h"
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include "dfs.h"
#include "feature_circle.h"
using namespace cv;
using namespace std;
int segmentation_roi(cv::Mat& roi, cv::Mat& mask, int feature, int method, std::vector< int >& param)
{
    if(param.size()<5){
        cout <<"two few arguments!"<<endl;
        return 0;
    }
    vector<StepInfo> path;
    vector<PathInfo> allpaths;
    int width = param.at(0);
    int height =  param.at(1);
    int direction[8][2] = {{width,height},{-width,height},{width,-height},{-width,-height},{height,width},{-height,width},{height,-width},{-height,-width}};
    //Mat visited = Mat(roi.size(),CV_8UC1,Scalar(0));
    
   
      Mat visited;
      visited = ~mask;
 
    switch(feature){
        case FEATURE_USE_CIRCLE:
        {
            vector<Point> centers;
            draws_circles(roi, visited,param,centers);
            dfs(visited,roi,mask,direction,feature,centers,param, path, allpaths);
            break;}
        case FEATURE_USE_CONTOUR:
            //vector<contour_feature>
            //draw_contour();
            break;
        case FEATURE_USE_TEXTURE:
           // draw_texture();
            break;
        default:
            cout <<"please choose a right feature"<<endl;
            //break;
    }



    
     /*   switch (method){
        case USE_DFS:
            dfs(visited,roi,mask,direction,feature,param, path, allpaths);
            break;
        case USE_BFS:
            // bfs(direction,roi.size(),centers,mean_radius,mean_object_dist,img,&count_segmentation);
            break;
        case USE_Dij:
            //
            break;
        default:
            cout <<"please choose a right method"<<endl;
            return 1;
    }*/
    printoutPath(roi,allpaths,direction);
    
    
   
    return 0;
}

void printoutPath(Mat& img,vector<PathInfo>& allpath, int direction[8][2])
{
    PathInfo bestpath;
    bool print_best_path = true;
    if(allpath.empty()){
        cout <<"no segmentation path have been found"<<endl;
    }else if(allpath.size()==1){
        cout << "just one path found!" <<endl;  
         bestpath = allpath.at(0);
        /*for(int j = 0;j<bestpath.path.size();j++){
            cout<<"path"<<bestpath.path.at(j).p<<"direction"<<bestpath.path.at(j).direction <<" score "<<bestpath.path.at(j).score<<endl; 
            } */
        
    }else{
        float score = get_mean_score(allpath.at(0).path);
        int index = 0;
        for(int i = 1;i<allpath. size();i++){
            //vector<Step> path = allpath.at(i).path; 
            float s = get_mean_score(allpath.at(i).path);
            cout<<"score"<<s<<endl;
           if(s>score) {index = i;score = s;}
        } 
        cout <<allpath.size()<<"paths have been found,return the best path"<< endl;
         bestpath = allpath.at(index);
    }
    
    
    
    if(print_best_path){
        for(int j = 0;j<bestpath.path.size();j++){
             Point p = bestpath.path.at(j).p;
             int d = bestpath.path.at(j).direction;
             Point c1(p.x+direction[d][0],p.y);
             Point c2(p.x,p.y+direction[d][1]);
             rectangle(img,Rect(c1,c2),Scalar(255));
             putText(img,to_string(j+1),p,FONT_HERSHEY_SCRIPT_SIMPLEX,2,Scalar(255));
            cout<<"path "<<p<<" direction "<<d<<" score "<<bestpath.path.at(j).score<<endl; 
        }
        imshowResize("segmentation result",img);
        waitKey(0);
        
    }else{
        for(int m = 0;m<allpath.size();m++){
            //m =;
            PathInfo path = allpath.at(m);
            for(int j = 0;j<path.path.size();j++){
                Point p = path.path.at(j).p;
                int d = path.path.at(j).direction;
                Point c1(p.x+direction[d][0],p.y);
                Point c2(p.x,p.y+direction[d][1]);
                rectangle(img,Rect(c1,c2),Scalar(255));
                putText(img,to_string(j+1),p,FONT_HERSHEY_SCRIPT_SIMPLEX,2,Scalar(255));
                cout<<"path "<<p<<" direction "<<d<<" score "<<path.path.at(j).score<<endl; 
            }
           
            imshowResize("segmentation result path "+to_string(m),img);
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


