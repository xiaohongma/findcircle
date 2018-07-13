#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <boost/mpl/accumulate.hpp>
#include <numeric>
using namespace cv;
using namespace std;
#define WINDOWNAME "original  image"
double sng(double x);
void imshowResize(const String& winname, Mat& img);
void selectROI(int event, int x, int y, int flags, void* img);
void saveROI(Mat& img, Rect rect);
void estimateCenterDistance(Mat& img, vector< Point > centers,double radius, Point* bl,double* mean_object_dist, double* angle);

void dfs(int direction[8][2], Mat& visited, vector< Point > centers, double radius, double mean_dist, Mat& img);
/**
@param direction[] is the search direction for next rectangle
@param basedPoint is the bottom left circle ceter point in current rectabgle
@param nextblpoint is the bottom left point in next rectangle. If  the remaining points cannot be formed into a m*n rect,
then, it will return current rect point again.
*/
bool extendAdjustRect(Mat& visited, vector< Point > centers, int direction[], Point basePoint, double radius, double dist, Point* nextblpoint, Rect* rectreturn, Mat& img);
void setVisited(Mat& visited, Point blpoint, int direction[], uchar a, double radius, double dist, vector< Point > centers, Mat& img);
bool isEnd(Mat& visited, vector< Point > centers);
void findblcenter(Mat& visited, vector<Point> centers, Point* bl);
bool g_bDrawingBox = false;
Rect g_rectangle = Rect(-1,-1,0,0);
Mat visited;
//prior: radius, n*n
//可乐21, 24*9 农夫11.7 28*10pixel
double assume_radius = 21;
//
double test_count = 24*9;
int m =6;//x
int n =4;//y
int count_segmentation ;
//void random_select_n(vector<Point>* input, int n, vector<Point>* output);
void AdaptiveFindThreshold(const CvArr* image, double* low, double* high, int aperture_size =3);
void _AdaptiveFindThreshold(CvMat *dx, CvMat *dy, double* low, double* high);
int main(int argc, char **argv) {
    Mat srcImage = imread("1_texture.jpg");
    //select ROI region(rectangle)
    imshowResize(WINDOWNAME,srcImage);
    setMouseCallback(WINDOWNAME, selectROI, (void*) &srcImage);
    waitKey(0);

    Mat distImg, tmpImg;
    Mat roi = imread("roi.jpg");
    distImg = roi;
    cvtColor(roi, tmpImg, CV_BGR2GRAY);
    GaussianBlur(tmpImg,tmpImg,Size(9,9),2,2);
    IplImage ipltmp = tmpImg;
    Mat edges;
    double canny_low = 0;
    double canny_high = 0;
    int acc_threshold = 30;// make the 2*n*n > count>= n*n

    int min_radius = 0.8*assume_radius;
    int max_radius = 1.2*assume_radius;
    int min_dist = 2*assume_radius;
    AdaptiveFindThreshold(&ipltmp, &canny_low, &canny_high);
    //cout <<"canny threshold"<<canny_high << endl;

    double canny_thresroud = canny_high;
    Canny(tmpImg,edges, canny_thresroud, canny_thresroud/2);
    imshowResize("canny edge",edges);



    vector<Vec3f> circles;
    //自适应寻找hough变换的阈值acc_threshold(这里设置步长为2，以后可以自适应步长)
    HoughCircles(tmpImg,circles,CV_HOUGH_GRADIENT,1,
                 min_dist,canny_thresroud,acc_threshold,min_radius,max_radius);
    //int try_hough = 1;
    while(1) {
        if(circles.size() < test_count) {
            acc_threshold = acc_threshold-1;
            HoughCircles(tmpImg,circles,CV_HOUGH_GRADIENT,1,
                         min_dist,canny_thresroud,acc_threshold,min_radius,max_radius);
            cout << "threshold:" << acc_threshold<<endl;
            cout << "count:" << circles.size()<<endl;
        } else if(circles.size() > 2*test_count) {
            acc_threshold = acc_threshold+1;
            HoughCircles(tmpImg,circles,CV_HOUGH_GRADIENT,1,
                         min_dist,canny_thresroud,acc_threshold,min_radius,max_radius);
            cout << "threshold:" << acc_threshold<<endl;
            cout << "count:"<< circles.size()<<endl;
        } else {


            break;
        }
        //try_hough++;
    }

    vector<Point> centers;
    vector<int> radiuses;
    for (int i = 0; i< circles.size(); i++) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        centers.push_back(center);
        int radius = cvRound(circles[i][2]);
        radiuses.push_back(radius);
        // circle(distImg,center, 5,Scalar(155,50,255),-1,4,0);
        // circle(distImg,center,radius,Scalar(0,255,0),2,4,0);
        // putText(distImg,to_string(i),center,0 ,1, Scalar(0,0,255),4,8);
    }
    double mean_radius =  std::accumulate(begin(radiuses),end(radiuses), 0.0)/radiuses.size();
    cout << "mean radius" << mean_radius <<endl;
    double mean_object_dist;
    Point bottom_left;
    double angle;
//根据顶点和圆心距建立n×m的模板
    estimateCenterDistance(distImg,centers,mean_radius,&bottom_left,&mean_object_dist, &angle);//圆心距离
    Point tl = Point(bottom_left.x-1.25*mean_radius, bottom_left.y-((n-1)*(mean_object_dist))*cos(-angle)-1.25*mean_radius);//左上

    cout << mean_object_dist<< endl;
    cout << bottom_left << endl;
    // plus the 2 times of mean_radius for width or height, since we also want to find the drifted centers.
    Size2f size = Size2f( mean_object_dist*(m-1)+2.5*mean_radius,mean_object_dist*(n-1)+2.5*mean_radius);
    Rect sub_rect = Rect(tl,size);//the first sub rectangle


// find the point in the rectangle
    vector<Point> centers_in_subrectangle;
    for( int i = 0; i< centers.size(); i++) {
        if(sub_rect.contains(centers[i])) {
            centers_in_subrectangle.push_back(centers[i]);

            // circle(distImg,centers[i], mean_radius,Scalar(0,255,0),1,4,0);
        }
        //  circle(distImg,centers[i], 5,Scalar(155,50,255),-1,4,0);
    }


    //deep first search to segment ROI into several rectangle
    int direction[8][2] = {{m-1,n-1},{n-1,m-1},{-(m-1),-(n-1)},{-(n-1),-(m-1)},{m-1,-(n-1)},{-(m-1),n-1},{n-1,-(m-1)},{-(n-1),m-1}};

    // visited map to record the visiting status of circle centers
    visited = Mat(distImg.size(),CV_8UC1,Scalar(255));
    for (int i = 0; i< centers.size(); i++) {

        visited.at<uchar>(centers.at(i)) = 0;

        //  circle(visited,p1, 5,Scalar(155,50,255),-1,4,0);
        //  circle(visited,p, 5,Scalar(155,50,255),-1,4,0);
    }
    Point point = bottom_left;
    dfs(direction,visited,centers,mean_radius,mean_object_dist,distImg);


    cout <<"count_segmentation" << count_segmentation<<endl;



    //cv::rectangle(distImg,sub_rect,Scalar(255),1,8,0);

    //imshowResize("first box",distImg);

    waitKey(0);

    return 0;
}

void imshowResize(const String& winname, Mat& img)
{
    namedWindow(winname, CV_WINDOW_NORMAL);
    cvResizeWindow(winname.c_str(), 700,700);
    imshow(winname, img);
}

void selectROI(int event, int x, int y, int flags, void* img)
{

    Mat& image = *(Mat*) img;
    switch( event ) {
    // 鼠标移动消息
    case EVENT_MOUSEMOVE: {
        if (g_bDrawingBox) {
            //绘制标识为真，则记录下长和宽到RECT变量中
            g_rectangle.width = x - g_rectangle.x;
            g_rectangle.height = y - g_rectangle.y;
        }
    }
    break;
    //左键按下消息
    case EVENT_LBUTTONDOWN: {
        g_bDrawingBox = true;
        g_rectangle = Rect(x, y, 0, 0);
        // cout << "Hello Button down" << endl;
    }
    break;
    //左键抬起消息
    case EVENT_LBUTTONUP: {
        g_bDrawingBox = false;
        if (g_rectangle.width < 0) {
            g_rectangle.x += g_rectangle.width;
            g_rectangle.width *= -1;
        }

        if (g_rectangle.height < 0) {
            g_rectangle.y += g_rectangle.height;
            g_rectangle.height *= -1;
        }
        saveROI(image, g_rectangle);


    }
    break;
    default:
        printf("error!\n");
    }
}
void saveROI(Mat& img, Rect rect)
{
    Mat roi = img(rect);
    imwrite("roi.jpg",roi);
    imshow("roi",roi);
    //rectangle(img, rect.tl(), rect.br(), Scalar(0, 255,0), 2, 1, 0);
    destroyWindow(WINDOWNAME);
    //imshowResize("original image",img);
}

void AdaptiveFindThreshold(const CvArr* image, double* low, double* high, int aperture_size )
{
    cv::Mat src = cv::cvarrToMat(image);
    // imshow("Adaptive", src);
    const int cn = src.channels();
    cv::Mat dx(src.rows, src.cols, CV_16SC(cn));
    cv::Mat dy(src.rows, src.cols, CV_16SC(cn));

    cv::Sobel(src, dx, CV_16S, 1, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);
    cv::Sobel(src, dy, CV_16S, 0, 1, aperture_size, 1, 0, cv::BORDER_REPLICATE);

    CvMat _dx = dx, _dy = dy;
    _AdaptiveFindThreshold(&_dx, &_dy, low, high);

}

// 仿照matlab，自适应求高低两个门限
void _AdaptiveFindThreshold(CvMat *dx, CvMat *dy, double* low, double* high)
{
    CvSize size;
    IplImage *imge=0;
    int i,j;
    CvHistogram *hist;
    int hist_size = 255;
    float range_0[]= {0,256};
    float* ranges[] = { range_0 };
    double PercentOfPixelsNotEdges = 0.7;
    size = cvGetSize(dx);
    imge = cvCreateImage(size, IPL_DEPTH_32F, 1);
    // 计算边缘的强度, 并存于图像中
    float maxv = 0;
    for(i = 0; i < size.height; i++ )
    {
        const short* _dx = (short*)(dx->data.ptr + dx->step*i);
        const short* _dy = (short*)(dy->data.ptr + dy->step*i);
        float* _image = (float *)(imge->imageData + imge->widthStep*i);
        for(j = 0; j < size.width; j++)
        {
            _image[j] = (float)(abs(_dx[j]) + abs(_dy[j]));
            maxv = maxv < _image[j] ? _image[j]: maxv; //保存最大的边缘强度

        }
    }
    if(maxv == 0) {
        high = 0;
        low = 0;
        cvReleaseImage( &imge );
        return;
    }

    // 计算直方图，bins范围是灰度范围，统计量为梯度值。选择梯度比例为0.7处的灰度值作为高阈值。
    range_0[1] = maxv;
    hist_size = (int)(hist_size > maxv ? maxv:hist_size);//选择bin的范围，如果灰度值达不到255，就用较小值
    hist = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
    cvCalcHist( &imge, hist, 0, NULL );
    int total = (int)(size.height * size.width * PercentOfPixelsNotEdges);
    float sum=0;
    int icount = hist->mat.dim[0].size;
    //cout <<icount <<endl;

    float *h = (float*)cvPtr1D( hist->bins, 0 );
    for(i = 0; i < icount; i++)
    {
        sum += h[i];
        if( sum > total )
            break;
    }
    // 计算高低门限
    *high = (i+1) * maxv / hist_size ;
    *low = *high * 0.5;
    cvReleaseImage( &imge );
    cvReleaseHist(&hist);
}

void estimateCenterDistance(Mat& img, vector< Point > centers, double radius, Point* bl, double* mean_object_dist, double* angle)
{
    RotatedRect rect = minAreaRect(centers);
    *angle = rect.angle;
    //cout << "angle"<< rect.angle <<endl;
    //取外接矩形四个定点处的n*n个圆，计算圆心距。
    Point2f vertices[4];
    rect.points(vertices);
    //vector<double> dist1(centers.size());
    //double min;
    // int index;
    Point2f corner_4[4];//距离外接矩形四个顶点最近的圆心
    for (int i = 0; i < 4; i++) {
        double  min_dist = 1000000;
        int  index = 0;
        double dist1 = 0;
        for(int j = 0; j< centers.size(); j++) {
            Point a = centers.at(j);
            //cout << (a.x -vertices[i].x)*(a.x -vertices[i].x) +(a.y -vertices[i].y)*(a.y -vertices[i].y)<<endl;
            dist1 = (a.x -vertices[i].x)*(a.x -vertices[i].x) +(a.y -vertices[i].y)*(a.y -vertices[i].y);
            //dist[j] = (centers[j][0]-vertices[i][0])^2 +(centers[j][1]-vertices[i][1])^2;
            if(dist1<min_dist) {
                min_dist = dist1;
                index = j;
            }
        }
        corner_4[i] = centers[index];
        //centers.clear();
        // circle(img,corner_4[i], 5,Scalar(155,50,255),-1,4,0);
        //line(img, vertices[i], vertices[(i+1)%4], Scalar(255));
    }


    vector<Point> selected_object(4);//角点附近的圆心
    vector<double> object_dist(4); //圆心距离
    //  double object_space[2];

    for (int i = 0; i < 4; i++) {
        double  min_dist = 1000000;
        int  index = 0;
        double dist1 = 0;
        //int num = 2;
        // while(1){
        for(int j = 0; j< centers.size(); j++) {
            Point a = centers.at(j);

            dist1 = (a.x -corner_4[i].x)*(a.x -corner_4[i].x) +(a.y -corner_4[i].y)*(a.y -corner_4[i].y);
            if(dist1 ==0) continue;

            if(dist1<min_dist) {
                min_dist = dist1;
                index = j;
            }
        }
        selected_object.at(i) = centers.at(index);
        object_dist.at(i) = sqrt(min_dist);
        // circle(img,selected_object[i], 5,Scalar(155,50,255),-1,4,0);
        cout << object_dist.at(i)<< endl;

    }

    //圆心距
    *mean_object_dist = accumulate(object_dist.begin(),object_dist.end(), 0.0)/object_dist.size();

    //calculate the top right point of sub-box
    *bl = Point(corner_4[0].x, corner_4[0].y);

    //*tl = Point(bl.x-1.25*radius, bl.y-((n-1)*(*mean_object_dist))*cos(-angle)-1.25*radius);//左上

    // circle(img,*tl,20,Scalar(0,255,0),-1);
    //*tl = Point(bl.x-((m-1)*(*mean_object_dist))*sin(-angle)- radius, bl.y-((m-1)*(*mean_object_dist))*cos(-angle)-radius);//左上角
    // circle(img,bl, 5,Scalar(0,0,255),-1,4,0);
    //    circle(img,*tl, 5,Scalar(0,0,255),-1,4,0);
    //*tl = Point(bl.x-((m-1)*(*mean_object_dist))*sin(-angle)- radius, bl.y-((m-1)*(*mean_object_dist))*cos(-angle)-radius);//左上角
    // *tl = Point(bl.x-((m-1)*(*mean_object_dist)+radius)*sin(-angle), bl.y-((m-1)*(*mean_object_dist)+radius)*cos(-angle));//左上角
    //建立矩形模型（比如4×6）
    // return mean_object_dist;

}

/*void random_select_n(vector< Point >* input, int n, vector< Point >* output)
{
  int size = input->size();
  vector<int> tmp1;
  for(int i =0; i<n; i++){
    output->push_back(input->at(rand()%size));
     //  cout << rand()%100 << endl;;
  }
}*/

bool extendAdjustRect(Mat& visited,vector<Point> centers,int direction[], Point basePoint, double radius,double dist, Mat& img)
{
    Point tl(basePoint.x-1.25*radius*sng(direction[0]),basePoint.y+ direction[1]*dist+ 1.25*radius*sng(direction[1]));

    Point br(basePoint.x+ direction[0]*dist+1.25*radius*sng(direction[0]), basePoint.y -1.25*radius*sng(direction[1]));

    // circle(visited, tl,10,Scalar(0,255,0));
    // circle(visited, br,10,Scalar(0,255,0));
    // Point tr(basePoint.x-1.25*radius+size.height,basePoint.y+1.25*radius-size.height);
    Rect rect = Rect(tl,br);
    //  rectangle(visited, rect, Scalar(0,255,0));
    Rect roi(Point(0,0),visited.size());
    // if the next rect is going to extend the roi bouding, we will return the current point.

    if((roi & rect)!= rect) {
        //*nextblpoint = basePoint;
        return false;

    }
    // the next rect contains less unvisited circle centers than m*n, we also return current point.
    int count = 0;
    for (int i = 0; i < centers.size(); i++) {
        if(rect.contains(centers.at(i)) && visited.at<uchar>(centers.at(i))==0)
            count++;
    }
    if(count < 24) {
        //*nextblpoint = basePoint;
        cout << count << endl;
        return false;
    }



    /* vector<Point> remaining;
     for (int i = 0; i < centers.size(); i++){
       if(!rect.contains(centers.at(i))){
    remaining.push_back(centers.at(i));
    //circle(img,centers.at(i),radius,Scalar(0,0,100),-1,4,0);
       }
     }
      if(remaining.empty()){
        return false;
     }*/

    return true;
    // *nextblpoint = remaining.at(index);


    /*int x_lb = 0;
    int y_lb = visited.rows;
    double distance = (remaining.at(0).x-x_lb)*(remaining.at(0).x-x_lb)+(remaining.at(0).y-y_lb)*(remaining.at(0).y-y_lb);
    int index;
    for(int i =1; i< remaining.size();i++){
      double d2 = (remaining.at(i).x-x_lb)*(remaining.at(i).x-x_lb)+(remaining.at(i).y-y_lb)*(remaining.at(i).y-y_lb);
      if(d2 < distance){
    distance = d2;
    index = i;

      }*/



}
void dfs(int direction[8][2], Mat& visited,vector<Point> centers, double radius, double mean_dist, Mat& img)
{
    // all of circle have been setted to flase
    if (isEnd(visited,centers)) {
        count_segmentation++;
        //  for(int i = 0; i < centers.size();i++){
//if(visited.at<uchar>(centers.at(i))==255){
//	  circle(img,centers.at(i),15,Scalar(255,255,255),-1);
//	}
//      }
        return;
    }
    Point point;// bottom left point
    findblcenter(visited,centers,&point);
    //cout << centers.size()<<endl;

    for (int i = 0; i<8; i++) {

        // Rect rect;
        // setVisited(visited,basePoint,direction);
        //if have not visited
        //如果周围可以有邻接矩形
        if(extendAdjustRect(visited, centers,direction[i],point, radius,mean_dist,img)) {

            setVisited(visited,point,direction[i],255,radius, mean_dist,centers,img);// set the m*n rect into flase
            // extendAdjustRect(visited, centers,direction[i],point, radius,mean_dist, &nextblpoint, &rect,img);//find the adjusting react
            // cout << "next point" << nextblpoint << endl;
            // cout << "next point value"<<(int)visited.at<uchar>(nextblpoint)<< endl;
            // if(nextblpoint.x==0 && nextblpoint.y==0){
            //   setVisited(visited,point,direction[i],0,radius, mean_dist,centers,img);//back to set it to 0
            //   cout << "next point value"<<(int)visited.at<uchar>(nextblpoint)<< endl;
            //   continue;
            // }
            //  circle(img,point,15,Scalar(0,0,255),-1,4,0);
            dfs(direction, visited,centers, radius, mean_dist,img);
            setVisited(visited,point,direction[i],0,radius, mean_dist,centers,img);//back to set it to 0

        }

    }




}
void setVisited(Mat& visited, Point blpoint, int direction[],uchar a, double radius, double dist, vector<Point> centers, Mat& img)
{
    //Mat visited_copy = visited;


    Point p1(blpoint.x-1.25*sng(direction[0])*radius, blpoint.y-1.25*radius*sng(direction[1]));
    Point p2(p1.x, p1.y + direction[1]*dist+ 2.5*radius*sng(direction[1]));
    Point p3(p1.x+direction[0]*dist+2.5*radius*sng(direction[0]), p1.y);
// Point p4(p1.x+direction[0]*dist+ 2.5*radius*sng(direction[0]), p1.y + direction[1]*dist+ 2.5*radius*sng(direction[1]));

    Rect rect(p2,p3);

    //rectangle(img, rect, Scalar(0,255,0));

    /*for (int i = 0; i < centers.size();i++){
      if(rect.contains(centers.at(i))){
        if(visited.at<uchar>(centers.at(i))==a){
    //cout << "i " << (int)visited.at<uchar>(centers.at(i))<<endl;
          return;
        }

          continue;


      }
    }*/

    for (int i = 0; i < centers.size(); i++) {
        if(rect.contains(centers.at(i))) {
            visited.at<uchar>(centers.at(i)) = a;

            circle(img,centers.at(i), radius,Scalar(a,a,a),-1,8,0);
            //LineTypes();

        }
    }


    rectangle(img,rect,Scalar(255,0,0),1,16);
    // circle(img,blpoint,10,Scalar(255,0,0),-1,16,0);
    imshowResize("xx",img);
    waitKey(0);
}
bool isEnd(Mat& visited, vector<Point> centers)
{
    // int count =0;
    for( int i = 0; i < centers.size(); i ++ ) {
        // int a = (int)visited.at<uchar>(centers.at(0));
        if(visited.at<uchar>(centers.at(i))==255) {
            //  count++;
            continue;
        }
        return false;
    }
    //cout << "success " <<count<< endl;
    return true;

}

double sng(double x) {
    return (x <0)? -1 : (x> 0);
}

void findblcenter(Mat& visited, vector<Point> centers, Point* bl) {

    vector<Point> remaining;
    for (int i = 0; i < centers.size(); i++) {
        if(visited.at<uchar>(centers.at(i))==0) {
            remaining.push_back(centers.at(i));
            //circle(img,centers.at(i),radius,Scalar(0,0,100),-1,4,0);
        }
    }
    if(remaining.empty()) {
        return;
    }


    int x_lb = 0;
    int y_lb = visited.rows;
    double distance = (remaining.at(0).x-x_lb)*(remaining.at(0).x-x_lb)+(remaining.at(0).y-y_lb)*(remaining.at(0).y-y_lb);
    int index;
    for(int i =1; i< remaining.size(); i++) {
        double d2 = (remaining.at(i).x-x_lb)*(remaining.at(i).x-x_lb)+(remaining.at(i).y-y_lb)*(remaining.at(i).y-y_lb);
        if(d2 < distance) {
            distance = d2;
            index = i;
        }
    }

    *bl = remaining.at(index);

}




