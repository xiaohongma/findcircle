#include "dfs.h"
using namespace std;


void dfs(int direction[8][2], Mat& visited,vector<Point> centers, double radius, double mean_dist, Mat& img, int* count_segmentation)
{
    // all of circle have been setted to flase
    if (isEnd(visited,centers)) {
        (*count_segmentation)++;
        return;
    }
    Point point;// bottom left point
    findblcenter(visited,centers,&point);
    //cout << centers.size()<<endl;

    for (int i = 0; i<8; i++) {
        //if have not visited
        //如果周围可以有邻接矩形
        if(extendAdjustRect(visited, centers,direction[i],point, radius,mean_dist,img)) {

            setVisited(visited,point,direction[i],255,radius, mean_dist,centers,img);// set the m*n rect into flase
            dfs(direction, visited,centers, radius, mean_dist,img,count_segmentation);
            setVisited(visited,point,direction[i],0,radius, mean_dist,centers,img);//back to set it to 0
        }

    }

}



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
    //waiting for modifcation
    if(count < 22) {
        //*nextblpoint = basePoint;
        cout << count << endl;
        return false;
    }


    return true;


}

void setVisited(Mat& visited, Point blpoint, int direction[],uchar a, double radius, double dist, vector<Point> centers, Mat& img)
{

    Point p1(blpoint.x-1.25*sng(direction[0])*radius, blpoint.y-1.25*radius*sng(direction[1]));
    Point p2(p1.x, p1.y + direction[1]*dist+ 2.5*radius*sng(direction[1]));
    Point p3(p1.x+direction[0]*dist+2.5*radius*sng(direction[0]), p1.y);
// Point p4(p1.x+direction[0]*dist+ 2.5*radius*sng(direction[0]), p1.y + direction[1]*dist+ 2.5*radius*sng(direction[1]));

    Rect rect(p2,p3);

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
//find the bottom center of circles
void findblcenter(Mat& visited, vector<Point> centers, Point* bl) {

    vector<Point> remaining;
    for (int i = 0; i < centers.size(); i++) {
        if(visited.at<uchar>(centers.at(i))==0) {
            remaining.push_back(centers.at(i));
            //circle(img,centers.at(i),radius,Scalar(0,0,100),-1,4,0);
        }
    }
    //waiting for modification
    if(remaining.empty()|| remaining.size()==1) {
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

