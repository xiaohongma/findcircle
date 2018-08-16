#ifndef SEGMENTATION
#define SEGMENTATION
#include "utils.h"
//using namespace cv;
//using namespace std;


/**
* @brief this function is used to divide the roi into several box according to the feature
* 
* @param roi p_roi: input image
* @param feature indicates whether or not we use each feature, it is stored in fixed order:circle,contour,texture
* @param param stores prior params we have known. it is also in fixed order.first two is the width and height of the box, the third and forth are the numbers of cans along each side. And the fifth is stored radius. 
*/
int segmentation_roi(cv::Mat& roi, cv::Mat& mask, int feature, std::vector< int >& param);

/**
* @brief this function is used to printout the best segmentation path
* 
* @param allpaths storage all segmentation paths
*/
void printoutPath(cv::Mat& img, std::vector< PathInfo >& allpath, bool only_show_best_path = true);


/**
* @brief get the mean score of a path
* 
* @param path p_path:... every step
* @return float: mean score of every step
*/
float get_mean_score(vector<StepInfo>& path);

#endif
