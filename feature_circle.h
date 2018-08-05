#ifndef FEA_CIRCLE
#define FEA_CIRCLE
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
/**
* @brief we use the circle feature to get the score of each step, now we just count circles in every patch as the score. If the score is too low, we will not visit next step. Next step, Combine with the returned score based on other features, we will get an overall score using function isContinue(vector<float> score) 
* 
* @param visited p_visited: visit map
* @param img p_img: score img
* @param direction p_direction:
* @param basePoint p_basePoint:bl point, i do not want to say
* @param params p_params: you can see the description in the reference fucntion
* @param score p_score: return score
*/
void extend_by_circle(cv::Mat& visited, cv::Mat& img, cv::Mat& mask, int direction[], cv::Point basePoint, std::vector< int >& params, std::vector< cv::Point >& centers, float* score, std::vector< cv::Point >& visit_points);
void circle_belong_rect( Point tlpoint, int direction[], double radius, vector<Point>& centers,Mat& img,vector<bool>& belong_rect);

void circle_belong_rect( Rect& rect, double radius, vector<Point>& centers,Mat& img,vector<bool>& belong_rect);

void eat_padding(Rect& inrect, Rect& range_rect,Mat& visited, Rect& outrect );
//count the points on the bounding of circle
//references: https://en.wikipedia.org/wiki/Midpoint_circle_algorithm 
void points_on_circle(Point center, int radius, vector<Point>& points);

void raster_circle(Point center,
         int radius, vector<Point>& points);
void draws_circles(Mat& img,  Mat& visited,vector<int> params,vector<Point>& centers);
#endif
