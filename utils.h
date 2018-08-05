#ifndef UTILS
#define UTILS
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

enum{
    USE_DFS = 10,
    USE_BFS =11,
    USE_Dij = 12
};
enum{
   FEATURE_USE_CIRCLE = 0,
   FEATURE_USE_CONTOUR = 1,
   FEATURE_USE_TEXTURE =2
};

struct StepInfo{
    Point p;//the keypoint of search step. we think it is the bottom left concern of the rectangle.
    int direction;//search direction
    float score;
};
struct PathInfo{
    vector<StepInfo> path;// restore the path
    float tag;// tag of the path, indicate the possibility that this path is right.
};

/**
 @param winname and img same with the function imshow(). This function will show image in 700*700 size.
 you can also change the value according to your display, or replace it with an auto resize method.
 */
void imshowResize(const String& winname, const Mat& img);
/**
 * estimate distance between circles
  @param centers is the detected circle centers
  @param mean_object_dist return the mean distance of two neighbor circles
  @param angle return the angle of minarea adjusting rect
 */
void estimateCenterDistance(vector< Point > centers,double* mean_object_dist, double* angle);
/**
 * this two methods are used to adaptive canny edge detection.
 */
void _AdaptiveFindThreshold(CvMat *dx, CvMat *dy, double* low, double* high);
void AdaptiveFindThreshold(const CvArr* image, double* low, double* high, int aperture_size =3);

/**
* @brief  This function is used to get the sign(+,-) of input value
* 
* @param x p_x:is a double value, if x is negative ,return -1; if positive return 1
* @return double
*/
int sng(double x);

#endif


