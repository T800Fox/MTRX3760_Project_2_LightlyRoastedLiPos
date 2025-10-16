#ifndef MTRX3760_OOGWAY_MAZESOLVER_UTILS_HPP_
#define MTRX3760_OOGWAY_MAZESOLVER_UTILS_HPP_

#include <math.h>
#include <string>
#include <algorithm>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

struct Point {
    double x;
    double y;
};

struct Pose2D {
    Point pos;
    double theta;
};

double dist2D(const Point &p1, const Point &p2);
Point projectPoint(const Point &p, double dist, double theta);


//Line segment defined as 2 points in global space
class LineSeg {
    public:
        LineSeg(const Point &p1, 
                const Point &p2);

        double point_seg_col(const Point &point) const;

        std::string to_string() const;

    private:
        Point p1;
        Point p2;
};


#endif