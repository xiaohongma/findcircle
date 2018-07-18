#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include "utils.h"
using namespace cv;
using namespace std;

/**
 * deep first search for segmentation
@param direction[] is the search direction for next rectangle
@param visited is a (U8C1) map record visiting status of every center point. At start, all of the 
center points are setted to 0, if it is visited, we set it to 255. When all of the center points 
are visited( setted to 255),we think we have found segmantion road.
@param centers is the detected point center
@param radius the mean radius
@param mean_dist is the mean dist of neighbor circle centers
@param img is roi image
*/
void dfs(int direction[8][2], Mat& visited, vector< Point > centers, double radius, double mean_dist, Mat& img,int* count_segmentation);
/**
 * this function is used to check whether there are rectangles can be extented.
 @param visited is a map record the visiting status.
 @param centers is all circle centers
 @param direction is the search direction
 @param basePoint is the bottom left point of probably existing rectangle
 @param radius mean radius
 @param dist mean dist of two adjusting circle center
 @param img roi image, in case we need breakpoint or debug
 */
bool extendAdjustRect(Mat& visited,vector<Point> centers,int direction[], Point basePoint, double radius,double dist, Mat& img);
/**
 * this is used to set visited or unvisited of centers in the extended rectangle.
 @param visited visited map record visiting status, same size with  roi image
 @param blpoint bottom left point, same with the input in extendAdjustRect()
 @param radius, direction, dist, centers, img,same with params in extendAdjustRect()
 @param a uchar value(0 or 255) is to be written into center point
 */
void setVisited(Mat& visited, Point blpoint, int direction[], uchar a, double radius, double dist, vector< Point > centers, Mat& img);
/**
 this function used to determine when we have visited all circles. Only if all the circles are visited, can we return true;
 @param visited nothing to say.
 @param centers same with the above
 */
bool isEnd(Mat& visited, vector< Point > centers);
/**
 it is used to find the bottom left circles in every dfs
 */
void findblcenter(Mat& visited, vector<Point> centers, Point* bl);
/**
 This function is used to get the sign(+,-) of input value
 @param x is a double value, if x is negative ,return -1; if positive return 1
 */
double sng(double x);
/**
 * @param belong_rect  fucntion is used to find the point that inner the rectangle or have intersection in the rectanglethe return value will be written into address of belong_rect. true means belong to the rect, false means not belong.
 */
void circle_belong_rect( Point blpoint, int direction[], double radius, double dist, vector<Point> centers,Mat& img,vector<bool>& belong_rect);






