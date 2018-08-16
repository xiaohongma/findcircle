#ifndef UTILS
#define UTILS
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

enum FEATURE{
   FEATURE_USE_CIRCLE = 0,
   FEATURE_USE_CONTOUR = 1,
   FEATURE_USE_TEXTURE =2
};
enum PARAMS{
    PARAM_WIDTH_LOC =0,
    PARAM_HEIGHT_LOC = 1,
    PARAM_M_LOC =2,
    PARAM_N__LOC  =3,
    PARAM_RADIUS_LOC  =4,
    PARAM_OPEN_SCALE_LOC = 5,
    PARAM_CLOSE_SCALE_LOC =6,
    PARAM_POLY_APPROX_SCALE_LOC  =7
};

struct StepInfo{
    Point p;//the keypoint of search step. we think it is the bottom left concern of the rectangle.
    int direction;//search direction
    RotatedRect r_rect;
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
/**
* @brief convert float to string, output given precision.
* 
* @param number p_number:...
* @param n p_n:...
* @return std::__cxx11::string
*/
string floatToString(float number, int n);
/**
* @brief rotate img according to give angle
* 
* @param read_path p_read_path:...
* @param out_path p_out_path:...
* @param angle p_angle:...
* @return int
*/
int rotateImg(cv::String read_path,cv::String out_path,float angle);
/**
* @brief draw rotated rect
* 
* @param img p_img:...
* @param rr p_rr:...
* @param color p_color:...
*/
void DrawRotatedRect(cv::Mat& img, cv::RotatedRect& rr, cv::Scalar color);
#endif


