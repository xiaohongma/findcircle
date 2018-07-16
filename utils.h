#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

/**
 @param winname and img same with the function imshow(). This function will show image in 700*700 size.
 you can also change the value according to your display, or replace it with an auto resize method.
 */
void imshowResize(const String& winname, Mat& img);
/**
  @param centers is the detected circle centers
  @param mean_object_dist return the mean distance of two neighbor circles
  @param angle return the angle of minarea adjusting rect
 */
void estimateCenterDistance(vector< Point > centers,double* mean_object_dist, double* angle);
