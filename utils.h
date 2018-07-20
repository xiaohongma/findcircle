#ifndef UTILS
#define UTILS
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

struct Step{
    Point p;//the keypoint of search step. for circle bins, it is bottom left center.
    int direction;//search direction
};
struct Path{
    vector<Step> path;// restore the path
    int tag;// tag of the path, indicate the possibility that this path is right.
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
* @brief this function is used to printout the best segmentation path
* 
* @param paths p_paths: the path vector
*/
void printoutPath(std::vector< Path >& allpath);
#endif


