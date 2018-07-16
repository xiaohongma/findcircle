#include <opencv2/imgproc/imgproc.hpp>
#include <queue>
using namespace cv;
using namespace std;
//breadth first search
void bfs(int direction[8][2], Mat& visited, vector< Point > centers, double radius, double mean_dist, Mat& img);
void findblcenter(vector<int> visited, vector<Point> centers,Size size, Point* bl);
bool isEnd(vector<int> visited, vector<Point> centers, Mat& img);
bool extendAdjustRect(vector<int> visited,vector<Point> centers,int direction[], Point basePoint, double radius,double dist, Size size,Mat& img, vector<int>* points);
void  bfs(int direction[8][2], Size size, vector<cv::Point> centers, double radius, double mean_dist, cv::Mat& img, int* count_segmentation);

