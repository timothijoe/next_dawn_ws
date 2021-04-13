#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <iostream>
#include <limits>
#include <cmath>
#include <float.h>
#include <algorithm>
#include <assert.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
//#include <geometry_msgs>
// #include "dataset/obstacle_info.h"

namespace util_ics_env
{


//typedef DBL_MAX =  std::numeric_limits<double>::max();
//constexpr double DBL_MAX = std::numeric_limits<Index>::max();
const double SAFE_DIST = 1.0;
const double EPSILON = 1.0e-6;
const size_t INVALID_INDEX = std::numeric_limits<size_t>::max();
//const double DBL_MAX =  std::numeric_limits<double>::max();

// sign funtion to determine value positive(1), negative(-1), or zero(0)
inline int sign(double x){
    return x > EPSILON? 1:(x<-EPSILON? -1:0);
}


// From now on is the point struct and operation
struct Point{
public:
    Point():x(0.0), y(0.0) {}
    Point(double xp, double yp): x(xp), y(yp){}
    friend Point operator + (const Point &a, const Point &b);
    friend Point operator - (const Point &a, const Point &b);
    friend Point operator * (const Point &a, const double &b);
    friend Point operator * (const double &b, const Point &a);
    friend Point operator / (const Point &a, const double &b);
    friend bool operator == (const Point& a, const Point& b);
    friend double xmult(const Point &a, const Point &b);
    friend double dmult(const Point &a, const Point &b);
    friend Point rot(const Point &a, double x);
    friend double length(const Point &a);
    friend double square(const double &a);

public:
    double x,y;
};
// add operation
Point operator + (const Point &a, const Point &b){
    return Point(a.x + b.x, a.y + b.y);
}
// minus operation
Point operator - (const Point &a, const Point &b) {
    return Point(a.x - b.x, a.y - b.y);
}
// left multiply'
Point operator * (const Point &a, const double &b){
    return Point(a.x * b, a.y * b);
}
// right multiply
Point operator * (const double &b, const Point &a){
    return Point(a.x * b, a.y * b);
}
Point operator / (const Point &a, const double &b){
    return Point(a.x / b, a.y / b);
}
// cross product
double xmult(const Point &a, const Point &b){
    return a.x * b.y - a.y * b.x;
}
// dot product
double dmult(const Point &a, const Point &b){
    return a.x * b.x + a.y * b.y;
}
// rotation of a vector
Point rot(const Point &a, double x){
    double tx = a.x, ty = a.y;
    return Point(tx * cos(x) - ty * sin(x), tx * sin(x) + ty * cos(x));
}
// length
double length(const Point &a){
    return sqrt(a.x * a.x + a.y * a.y);
}
// get a square of a Point struct
double square(const double a){
    return pow(a,2);
}
// equal with a loose cretiea
bool operator == (const Point& a, const Point& b){
    return (fabs(a.x - b.x) < EPSILON && fabs(a.y - b.y) < EPSILON);
}


/*
 * segment definiftion
 *
**/
struct Segment
{
public:
    Point start, end;
    Segment(): start(0,0), end(0,0){}
    Segment(const Point& xp, const Point& yp): start(xp), end(yp){}

    friend Point getVectorNorm(const Segment& l);
    friend double calPointToLineDist(const Point& p, const Segment& l);
    friend double testPointOnLine(const Point& p, const Segment& l);

    friend bool testParallel(const Segment& lm, const Segment& ln);
    friend bool testIntersected(const Segment& lm, const Segment& ln, Point& p);
};


inline double calLength(const Segment& l) {
    return sqrt(square(l.start.x - l.end.x) + square(l.start.y - l.end.y));
}

inline Point getVectorNorm(const Segment& l){
    auto dist = calLength(l);
    if (sign(dist) == 0){
        throw std::runtime_error("getVectorNorm: dist equals zero!");
    }
    return (l.end - l.start) / dist;
}

/*
 * test whether a point is on a line
 *
**/
inline double testPointOnLine(const Point& p, const Segment& l){
    return sign(xmult(p-l.start, l.end-l.start)) == 0 &&
            sign(dmult(p-l.start, p-l.end)) <= 0;
}

/*
 * calculate the distance between a point and a segment
 * Note: it is not a infinite line
 *
**/
double calPointToLineDist(const Point& p, const Segment& l)
{
    if(sign(dmult(p - l.start, l.end - l.start)) < 0){
        return sqrt(square(p.x - l.start.x) + square(p.y - l.start.y));
    }
    else if(sign(dmult(p - l.start, l.end - l.start)) == 0){
        double dist_pa = sqrt(square(p.x - l.start.x) + square(p.y - l.start.y));
        double dist_pb = sqrt(square(p.x - l.end.x) + square(p.y - l.end.y));
        return dist_pa <= dist_pb ? dist_pa : dist_pb;
    }
    if(sign(dmult(p - l.end, l.start - l.end)) < 0){
        return sqrt(square(p.x-l.end.x) + square(p.y-l.end.y));
    }
    return fabs(xmult(l.start-p, l.end-p) /
                sqrt(square(l.start.x-l.end.x) + square(l.start.y-l.end.y)));
}


/*
 * whether two line segment are parallel or not
 * include the overlapped cases
 *
**/
inline bool testParallel(const Segment& lm, const Segment& ln){
    return !sign(xmult(lm.start - lm.end, ln.start - ln.end));
}


/*
 * whether two line segment intersects or not
 * return false if a and b are parallel
 *
 *
**/
bool testIntersected(const Segment& lm, const Segment& ln, Point& p)
{

    // include overlapped
    bool isParallel = testParallel(lm,ln);

    // isLnMissed == 0: one end of ln is in lm
    bool isLnMissed = xmult(lm.start-lm.end, lm.start-ln.start) * xmult(lm.start-lm.end, lm.start-ln.end) > 0;
    bool isLmMissed = xmult(ln.start-ln.end, ln.start-lm.start) * xmult(ln.start-ln.end, ln.start-lm.end) > 0;

    if (isParallel || isLnMissed || isLmMissed){
        return false;

    }else {
        double s1 = xmult(lm.start - ln.start, ln.end - ln.start);
        double s2 = xmult(lm.end - ln.start, ln.end - ln.start);

        // s1 - s2 will never be zero since a b are intersected
        // s1 - s2 = (lm.a - lm.b) cross product (ln.a - ln.b)
        p = (s1 * lm.end - s2 * lm.start) / (s1 - s2);
        return true;
    }

}




/*
 * Triangle Definition.
 * Point a is the origin of the shared edge;
 * Point b is the destination of the shared edge;
 * Point c is the another third edge;
 * Edge shared_eg is Line(a,b)
 * Edge next_eg is Line(b,c)
 * Edge pre_eg is Line(c,a)
 *
**/

class Circle{
public:
    struct Attribute{
        double radius;
        Point center;
    }attribute;
    int id;
    // Circle(const dataset::obstacle_info& obstacle){
    //     //attribute.orientation = (double)obstacle.pose.orientation.z;
    //     id = obstacle.id.data;
    //     double x = (double)obstacle.pose.position.x;
    //     double y = (double)obstacle.pose.position.y;
    //     attribute.center = Point(x,y);

    // }
    Circle(){
        //attribute.orientation = (double)obstacle.pose.orientation.z;
        id = 0;
        attribute.center = Point(0,0);

    }
    Circle(double x, double y){
        //attribute.orientation = (double)obstacle.pose.orientation.z;
        id = 0;
        attribute.center = Point(x,y);

    }
    Circle(int idd , double x, double y){
        //attribute.orientation = (double)obstacle.pose.orientation.z;
        id = idd;
        attribute.center = Point(x,y);

    }

    bool collision_checking(Point& point,double threshold);

};

bool Circle::collision_checking(Point& point, double threshold = 1){
    //double threshold = 1;
    //std::cout << length(this->attribute.center - point)<< " ge ming bi sheng" << std::endl; 
    //bool zt_label =  length(this->attribute.center - point) < threshold? true:false;
    //std::cout << zt_label << " hhh "<<std::endl;
    return length(this->attribute.center - point) < threshold? true:false;
}












class Rectangle{
public:
    struct Attribute{
        double width;
        double length;
        double orientation;//-pi-pi
        Point center;
    }attribute;

    Point vertex[4];
    Segment edge[4];
    int id;
    // Rectangle(const dataset::obstacle_info& obstacle){
    //     attribute.length = (double)obstacle.attribute[0].data;
    //     attribute.width = (double)obstacle.attribute[1].data;
    //     attribute.orientation = (double)obstacle.pose.orientation.z;
    //     id = obstacle.id.data;
    //     double x = (double)obstacle.pose.position.x;
    //     double y = (double)obstacle.pose.position.y;
    //     attribute.center = Point(x,y);
    //     Point tempp = Point(-attribute.length/2, attribute.width/2);
    //     //std::cout<< tempp.x<<tempp.y<<std::endl;
    //     vertex[0] = tempp + rot(tempp, attribute.orientation);
    //     tempp = Point(-attribute.length/2, -attribute.width/2);
    //     vertex[1] = tempp + rot(tempp,attribute.orientation);
    //     tempp = Point(attribute.length/2, -attribute.width/2);
    //     vertex[2] = tempp + rot(tempp,attribute.orientation);
    //     tempp = Point(attribute.length/2, attribute.width/2);
    //     vertex[3] = tempp + rot(tempp,attribute.orientation);
    //     edge[0] = Segment(vertex[0], vertex[1]);
    //     edge[1] = Segment(vertex[1], vertex[2]);
    //     edge[2] = Segment(vertex[2], vertex[3]);
    //     edge[3] = Segment(vertex[3], vertex[0]);
        
    //         //std::cout<< vertex[0].x << vertex[0].y << vertex[2].x << vertex[2].y<<std::endl;
    //     //std::cout<< x<<y<<std::endl;
        
    // }
        Rectangle(double length, double width, double xx, double yy, double rotation){
        attribute.length = length;
        attribute.width = width;
        attribute.orientation = rotation;
        double x = xx;
        double y = yy;
        attribute.center = Point(x,y);

        Point tempp = Point(-attribute.length/2, attribute.width/2);
        vertex[0] = attribute.center + rot(tempp, attribute.orientation);
        tempp = Point(-attribute.length/2, -attribute.width/2);
        vertex[1] = attribute.center + rot(tempp,attribute.orientation);
        tempp = Point(attribute.length/2, -attribute.width/2);
        vertex[2] = attribute.center + rot(tempp,attribute.orientation);
        tempp = Point(attribute.length/2, attribute.width/2);
        vertex[3] = attribute.center + rot(tempp,attribute.orientation);
        edge[0] = Segment(vertex[0], vertex[1]);
        edge[1] = Segment(vertex[1], vertex[2]);
        edge[2] = Segment(vertex[2], vertex[3]);
        edge[3] = Segment(vertex[3], vertex[0]);
        std::cout << "Rectangle vertex point coordinate"<<std::endl;
        std::cout << "center ("<< attribute.center.x<< ","<< attribute.center.y<<")" <<std::endl;
        std::cout << "rotate" <<attribute.orientation <<std::endl;
        std::cout << vertex[0].x <<" "<<vertex[0].y<<" "<<vertex[2].x<<" "<<vertex[2].y<<std::endl;
        
    }
    bool priliminary_collision_checking(Point& point);
    //bool point_inside_checking(Point& point);
    bool fine_collision_checking(Point& point,double threshold);
    bool collision_checking(Point& point);

};

//return false if it gurantee not collision
//return true for more specific checking
bool Rectangle::priliminary_collision_checking(Point& point){
    double threshold = length(Point(this->attribute.width,this->attribute.length))/2;

    return length(this->attribute.center - point) < threshold? true:false;

}
/*
bool Rectangle::point_inside_checking(Point& point){
    bool indicator[4];
    for (size_t i = 0; i< 4; i++){
        Point line_1 = this->vertex[(i+1)%4] - this->vertex[i];
        Point line_2 = point - this->vertex[i];
        indicator[i] = sign(xmult(line_1,line_2))>0? true:false;       
    }
    return(indicator[0] && indicator[1] && indicator[2] && indicator[3]);

}
*/
bool Rectangle::fine_collision_checking(Point& point, double threshold = 0.2){
    bool indicator[4];
    for (size_t i = 0; i< 4; i++){
        Point line_1 = this->vertex[(i+1)%4] - this->vertex[i];
        Point line_2 = point - this->vertex[i];
        indicator[i] = sign(xmult(line_1,line_2))>0? true:false;       
    }
    if(indicator[0] && indicator[1] && indicator[2] && indicator[3]){
        //std::cout<<"number: 4"<<std::endl;
        return true;
    }
    else{
        double min_dist = DBL_MAX;
        for(size_t i = 0; i <4; i++){
            min_dist = std::min(min_dist, calPointToLineDist(point, edge[i]));
        }
        bool collision = min_dist < threshold? true:false;
        if(collision)
            std::cout<<"min_dist"<<min_dist<<std::endl;
        return collision;
    }
}

bool Rectangle::collision_checking(Point& point){
    if(!priliminary_collision_checking(point)){
        //std::cout<<"number: 1"<<std::endl;
        return false;
    }
    else if(!fine_collision_checking(point)){
        return false;
        //std::cout<<"number: 2"<<std::endl;
    }
    else{//std::cout<<"number: 3"<<std::endl;;
        return true;}

}
class Triangle{

public:
    Point vertex[3];
    Segment edge[3];

    Triangle(const Point& m, const Point& n, const Point& o){
        vertex[0] = m;
        vertex[1] = n;
        vertex[2] = o;
        edge[0] = Segment(vertex[0],vertex[1]);
        edge[1] = Segment(vertex[1],vertex[2]);
        edge[2] = Segment(vertex[2],vertex[0]);
    }

    size_t queryVertexIndex(const Point& a);
    size_t nearestVertexToLine(const Segment& l);
    void generateAnchors(const size_t index, std::vector<Point>& anchors);
};

/*
 * convert a vertex coordinate to its index
 *
**/
size_t Triangle::queryVertexIndex(const Point& a){
    size_t id = INVALID_INDEX;
    for(size_t i = 0; i < 3; ++i)
    {
        if(this->vertex[i] == a){
            id = i;
            return id;
        }
    }
    return id;
}

/*
 * get the nearest triangel vertex to a target line
 *
**/
size_t Triangle::nearestVertexToLine(const Segment &l){
    size_t i0 = 0;
    double min_dist = DBL_MAX;
    for (size_t i = 0; i < 3; i++) {
        // squared distance
        double d = calPointToLineDist(this->vertex[i], l);
        if (d < min_dist) {
            i0 = i;
            min_dist = d;
        }
    }
    return i0;
}

/*
 * generate two anchors from adjacent edges for a target vertex
 * the distance to the vertex is set to 1 meter
 *
**/
void Triangle::generateAnchors(const size_t id, std::vector<Point>& anchors){

    anchors.clear();
    Point cur_point = this->vertex[id];

    // std::cout << cur_point.x << std::endl;
    // std::cout << cur_point.y << std::endl;

    auto& edge_0 = this->edge[id];
    auto& edge_1 = this->edge[(id+2)%3];

    // select the middle point if less than safe distance
    if (calLength(edge_0) < SAFE_DIST){
        anchors.push_back((edge_0.start + edge_0.end) * 0.5);
    }else {
        anchors.push_back(cur_point + SAFE_DIST * getVectorNorm(edge_0));
    }

    if (calLength(edge_1) < SAFE_DIST){
        anchors.push_back((edge_1.start + edge_1.end) * 0.5);
    }else {
        // the edge is in counter clock wise - the sign is '-' here
        anchors.push_back(cur_point - SAFE_DIST * getVectorNorm(edge_1));
    }

}


/*
 * Segment: start - robot position, end - goal position
 *
**/
void placeNode(Triangle& cur_triangle, const Segment se_line, Point& placement) {

    // test geometry relationship
    std::vector<size_t> inter_edges;
    std::vector<size_t> paral_edges;
    std::vector<Point> inter_points;

    for (size_t i = 0; i < 3; i++) {
        // whether intersected with the edges
        Point point;
        if (testIntersected(se_line, cur_triangle.edge[i], point)){
            inter_edges.push_back(i);
            inter_points.push_back(point);
        }
        // whether parallel with the edges
        if (testParallel(se_line, cur_triangle.edge[i])){
            paral_edges.push_back(i);
        }
    }
    auto paral_count = paral_edges.size();
    auto inter_count = inter_edges.size();
    if (paral_count > 1){
        std::cout << cur_triangle.vertex[0].x << std::endl;
        std::cout << cur_triangle.vertex[0].y << std::endl;
        std::cout << cur_triangle.vertex[1].x << std::endl;
        std::cout << cur_triangle.vertex[1].y << std::endl;
        std::cout << cur_triangle.vertex[2].x << std::endl;
        std::cout << cur_triangle.vertex[2].y << std::endl;

        std::cout << se_line.start.x << std::endl;
        std::cout << se_line.start.y << std::endl;

        std::cout << se_line.end.x << std::endl;
        std::cout << se_line.end.y << std::endl;


        throw std::runtime_error("PlaceNode: parallel count > 1!");
    }


    // different intersection cases (plot on the page please)
    if (inter_count == 0){ // paral_count == 0 or 1
        // std::cout << "(inter = 0)" << std::endl;

        // no intersection with triangles
        std::vector<Point> anchors;
        auto idx = cur_triangle.nearestVertexToLine(se_line);
        cur_triangle.generateAnchors(idx, anchors);
        placement = (anchors[0] + anchors[1]) * 0.5;

    } else if (inter_count == 1) {  // paral_count == 1 (0 is impossible)
        // std::cout << "(inter = 1)" << std::endl;
        // the robot arrives at goal triangle
        placement = se_line.end;
        if (!(placement == inter_points[0])){
            throw std::runtime_error("PlaceNode: incorrect segment format!");
        }

    } else if (paral_count == 1 && inter_count == 2) {
        std::cout << "(paral = 1 && inter = 2)" << std::endl;

        // test whether the anchor is a triangle vertex
        auto idx_0 = cur_triangle.queryVertexIndex(inter_points[0]);

        // pass through two edges
        if(idx_0 == INVALID_INDEX){
            placement = (inter_points[0] + inter_points[1]) * 0.5;
        }
        // intersect on a vertex
        else if (inter_points[0] == inter_points[1]){
            std::vector<Point> anchor_points;
            cur_triangle.generateAnchors(idx_0, anchor_points);
            placement = (anchor_points[0] + anchor_points[1]) * 0.5;
        }
        // overlapped with one edge
        else if (!(inter_points[0] == inter_points[1])){
            // the first candidate placement
            std::vector<Point> anchor_points;
            cur_triangle.generateAnchors(idx_0, anchor_points);
            auto candidte_0 = (anchor_points[0] + anchor_points[1]) * 0.5;
            // the second candidate placement
            anchor_points.clear();
            auto idx_1 = cur_triangle.queryVertexIndex(inter_points[1]);
            cur_triangle.generateAnchors(idx_1, anchor_points);
            auto candidte_1 = (anchor_points[0] + anchor_points[1]) * 0.5;
            // average the candidates
            placement = (candidte_0 + candidte_1) * 0.5;
        }
        else {
            throw std::runtime_error("PlaceNode: unexpected result in case-4!");
        }

    } else if (paral_count == 0 && inter_count == 3){
        // pass through one edge and the opposite vertex
        // std::cout << "(paral = 0 && inter = 3)" << std::endl;
        // anchor_points[2] is the intersaction with the edge
        if (inter_points[0] == inter_points[1]){
            placement = (inter_points[0] + inter_points[2]) * 0.5;
        } else {
            placement = (inter_points[0] + inter_points[1]) * 0.5;
        }

    } else if (paral_count == 0 && inter_count == 2){
        // std::cout << "(paral = 0 && inter = 2)" << std::endl;

        // intersect with a vertex
        if (inter_points[0] == inter_points[1]){
            // the vertex is goal point
            if (inter_points[0] == se_line.end){
                placement = se_line.end;
            } else { // not a goal point
                size_t idx = cur_triangle.queryVertexIndex(inter_points[0]);
                if (idx == INVALID_INDEX){
                    throw std::runtime_error("PlaceNode: unexpected index!");
                }
                std::vector<Point> anchors;
                cur_triangle.generateAnchors(idx, anchors);
                placement = (anchors[0] + anchors[1]) * 0.5;
            }

        }else {// intersect with two edges
            placement = (inter_points[0] + inter_points[1]) * 0.5;
        }

    } else {
        throw std::runtime_error("PlaceNode: unexpected cases!");
    }

} // end placeNodeEx


}

#endif
