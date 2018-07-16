#include "bfs.h"
#include "utils.h"
#include <iostream>

// considering tha
void  bfs(int direction[8][2], Size size, vector<cv::Point> centers, double radius, double mean_dist, cv::Mat& img, int* count_segmentation)
{
     queue<vector<int>> q;
    vector<int> path0;
    //Point bl0;
    //path0.push_back(bl0);
    q.push(path0);
    
    while(!q.empty()){
        vector<int> path = q.front();
        q.pop();
        Point bl ;
        vector<int> contain_points;
        findblcenter(path,centers,size,&bl);
        if(isEnd(path,centers,img)){
            (*count_segmentation)++;
                continue;
            }
    
        for(int i =0; i <8; i++){

            if(extendAdjustRect(path,centers,direction[i],bl,radius,
                mean_dist,size,img,&contain_points)){
                vector<int> newpath(path);
                for(int i = 0; i<contain_points.size();i++)
                newpath.push_back(contain_points.at(i));
                q.push(newpath);
             //   setVisited(visited,bl,direction[i],255,radius, mean_dist,centers,img);// set the m*n
            }
            
        }
        
        
    }
    
}




double sng2(double x) {
    return (x <0)? -1 : (x> 0);
}

//find center
//find the bottom center of circles
void findblcenter(vector<int> visited, vector<Point> centers,Size size, Point* bl) {
    vector<int> all_points;
    for(int i = 0; i<centers.size();i++){
        all_points.push_back(i);
    }
    sort(all_points.begin(),all_points.end());
    sort(visited.begin(),visited.end());
    vector<int> re;
    set_difference(all_points.begin(),all_points.end(),visited.begin(),visited.end(),back_inserter(re));

    vector<Point> remaining;
    for(int i = 0; i < re.size();i++){
    
        remaining.push_back(centers.at(re.at(i)));
    }
    //waiting for modification
    if(remaining.empty()|| remaining.size()==1) {
        return; 
    }


    int x_lb = 0;
    int y_lb = size.height;
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

bool isEnd(vector<int> visited, vector<Point> centers, Mat& img)

{      /* int x = rand();
        for(int i =0; i<visited.size();i++){
            circle(img,centers.at(visited.at(i)),10,Scalar(x%255,x%255,x%255),-1);
            
        }
        imshowResize("xx",img);
        waitKey(0);*/
    if(visited.size()==centers.size()){

        return true;
    }
    return false;


}

//extend rectangle
bool extendAdjustRect(vector<int> visited,vector<Point> centers,int direction[], Point basePoint, double radius,double dist, Size size,Mat& img, vector<int>* points)
{
    Point tl(basePoint.x-1.25*radius*sng2(direction[0]),basePoint.y+ direction[1]*dist+ 1.25*radius*sng2(direction[1]));

    Point br(basePoint.x+ direction[0]*dist+1.25*radius*sng2(direction[0]), basePoint.y -1.25*radius*sng2(direction[1]));


    Rect rect = Rect(tl,br);
    //  rectangle(visited, rect, Scalar(0,255,0));
    Rect roi(Point(0,0),size);
    // if the next rect is going to extend the roi bouding, we will return the current point.

    if((roi & rect)!= rect) {
        //*nextblpoint = basePoint;
        return false;

    }
    // the next rect contains less unvisited circle centers than m*n, we also return current point.
    
    vector<int> all_points;
    for(int i = 0; i<centers.size();i++){
        if(rect.contains(centers.at(i)))
        all_points.push_back(i);
    }
    sort(all_points.begin(),all_points.end());
    sort(visited.begin(),visited.end());
    vector<int> re;
    // the point belong to the first vector, but not belong to the second vector
    set_difference(all_points.begin(),all_points.end(),
                   visited.begin(),visited.end(),back_inserter(re));
    
    int count = re.size();

    //waiting for modifcation
    if(count < 22) {
        //*nextblpoint = basePoint;
        cout << count << endl;
        return false;
    }
    *points = re;

    return true;


}




