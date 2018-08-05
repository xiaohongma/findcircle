#ifndef DFS
#define DFS
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include "utils.h"
#include "feature_circle.h"
using namespace cv;
using namespace std;

/**
 * deep first search for segmentation
@param direction[] is the search direction for next rectangle
@param visited is a (U8C1) map record visiting status of every center point. At start, all of the 
center points are setted to 0, if it is visited, we set it to 255. When all of the center points 
are visited( setted to 255),we think we have found segmantion road.
@param img is roi image
@param feature indicates whether or not we use each feature, it is stored in fixed order:circle,contour,texture
@param param stores prior params we have known. it is also in fixed order.first two is the width and height of the box, the third and forth are the numbers of cans along each side. And the fifth is stored radius. 
@param path save current path
@param allpaths save allpaths
*/
template <typename T> void dfs(cv::Mat& visited, cv::Mat& img, cv::Mat& mask,int direction[8][2], int feature,vector<T>& units, std::vector<int>& params, std::vector<StepInfo>& path, std::vector<PathInfo>& allpaths);


/**
 * this function is used to check whether there are rectangles can be extented.
 @param visited is a map record the visiting status.
 @param feature indicates whether or not we use each feature, it is stored in fixed order:circle,contour,texture
 @param param stores prior params we have known. it is also in fixed order.first two is the width and height of the box, the third and forth are the numbers of cans along each side. And the fifth is stored radius. 
 @param direction is the search direction
 @param basePoint is the bottom left point of probably existing rectangle
 @param img roi image, in case we need breakpoint or debug
 @param score return the score in this step.
 */
template <typename T> void extendAdjustRect(cv::Mat& visited, cv::Mat& img, cv::Mat& mask, int direction[], cv::Point basePoint,int feature,vector<T>& units,std::vector< int >& params, float* score,vector<Point>& visit_points);


/**
 * this is used to set visited or unvisited of centers in the extended rectangle.
 @param visited visited map record visiting status, same size with  roi image
 @param point bottom left point, same with the input in extendAdjustRect()
 @param img, direction  the same with corresponding params in function extendAdjustRect()
 @param value uchar value(0 or 255) is to be written into center point
 */

void setVisited(cv::Mat& visited, cv::Mat& img,vector<Point>& visit_points,uchar value);
/**
 this function used to determine when we have visited whole img. 
 @param visited nothing to say.
 @param path, allpath, same with the above
 */
template <typename T>  bool isEnd(Mat& visited,vector<int>& params,vector<T>& units  );
template <typename T>  bool isEnd( cv::Mat& visited, std::vector<int>& params, std::vector<T>& units)
{

    bool isend = false;
    int m = params.at(2);
    int n = params.at(3);
    int count = 0;
    
    for(int i = 0;i<units.size();i++){
        if(visited.at<uchar>(units.at(i))==0) count++;
    }

    if(count<m*n*0.5){
        cout<<"unvisited areaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa  "<<count << endl;
        isend = true;
    }

    return isend;
}



void findtl(cv::Mat& visited, cv::Mat& img,cv::Point* point);

/**
* @brief decide whether or not we continue to visit
* 
* @param score p_score:
* @return bool 
*/
bool isContinue(float score);

void intersection_rect_mask(cv::Rect& rect, cv::Mat& mask, float* ratio);
void find_start_point(cv::Mat& visited, cv::Mat& img,cv::Point* point);



template <typename T> void dfs(cv::Mat& visited, cv::Mat& img, cv::Mat& mask,int direction[8][2], int feature,vector<T>& units, std::vector<int>& params, std::vector<StepInfo>& path, std::vector<PathInfo>& allpaths)
{
     // all of circle have been setted to flase
    //int num = abs(direction[0][1])*abs(direction[0][0]);
    if (isEnd(visited,params,units)) {
        //(*count_segmentation)++;
        PathInfo pathInfo;
        pathInfo.path=path;
        pathInfo.tag = 123;
        
        allpaths.push_back(pathInfo);
        return;
    }
    Point point(-1,-1);// top left point
   // findtl(visited,img,&point); 
    find_start_point(visited,img,&point);
    cout<<"dfs st2"<<point<<endl;
    if(point.x==-1 && point.y==-1) return;

    for (int i = 0; i<8; i++) {
         float score=0;
         vector<Point> visit_points;
        //if have not visited
        extendAdjustRect(visited,img,mask,direction[i],point,feature,units, params,&score,visit_points);
        //如果周围可以有邻接矩形
        if(isContinue(score)) {
            StepInfo step;
            //circle(img,point,5,Scalar(255),-1);
            // set the m*n rect into flase
           setVisited(visited,img,visit_points,255);
          // cout <<"visited direction" <<i<<" visited bl"<<point<<endl;
           step.direction = i;
           step.p = point;
           step.score = score;
           path.push_back(step);
            dfs(visited,img,mask,direction,feature,units,params,path,allpaths);
            setVisited(visited,img,visit_points,0);
           // setVisited(visited,img,mask,bounding,0);//back to set it to 0
            path.pop_back();
        }

    }

}


template<typename T>void extendAdjustRect(cv::Mat& visited, cv::Mat& img, cv::Mat& mask, int direction[], cv::Point basePoint, int feature, std::vector< T >& units, std::vector< int >& params, float* score, std::vector< cv::Point >& visit_points)
{
    switch(feature){
        case FEATURE_USE_CIRCLE:
             extend_by_circle(visited,img,mask,direction,basePoint,params,units,score,visit_points);
            break;
        case FEATURE_USE_CONTOUR:
            //score_by_contour(visited,img,direction,basePoint,params,score);
            break;
        case FEATURE_USE_TEXTURE:
            //score_by_texture(visited,img,direction,basePoint,params,score);
            break;
        default:
            cout <<"please choose a right feature"<<endl;
            //break;
    }
    
}


#endif





