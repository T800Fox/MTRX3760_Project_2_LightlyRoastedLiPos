#include "mtrx3760_oogway_mazeSolver/utils.hpp"


double dist2D(const Point &p1, const Point &p2){
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

Point projectPoint(const Point &p, double dist, double theta){
    return Point{p.x + sin(theta) * dist, p.y + cos(theta) * dist};
}


LineSeg::LineSeg(const Point &p1, const Point &p2)
                :p1(p1), p2(p2){}


//ChatGPT Generated function
double LineSeg::point_seg_col(const Point &point) const{
    double vx = p2.x - p1.x;
    double vy = p2.y - p1.y;
    double wx = point.x - p1.x;
    double wy = point.y - p1.y;

    double len2 = vx*vx + vy*vy;
    double t = (wx*vx + wy*vy) / len2; // projection scalar

    t = std::clamp(t, 0.0, 1.0);
    double proj_x = p1.x + t * vx;
    double proj_y = p1.y + t * vy;
    double dx = point.x - proj_x;
    double dy = point.y - proj_y;
    return std::sqrt(dx*dx + dy*dy);

}



std::string LineSeg::to_string() const{
    return "LineSeg: (" + std::to_string(p1.x) + ", " + std::to_string(p1.y) +
               ") -> (" + std::to_string(p2.x) + ", " + std::to_string(p2.y) + ")";
}