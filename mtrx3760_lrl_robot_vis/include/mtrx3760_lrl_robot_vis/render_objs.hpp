#ifndef ROBOT_VIS_RENDER_OBJS_HPP_
#define ROBOT_VIS_RENDER_OBJS_HPP_

#include <SDL.h>
#include <cmath>
#include <string>
#include <iostream>

//Colour consts
const SDL_Color BG_colour = {30,30,30,255};
const SDL_Color Grid_colour = {30,30,30,255};

// Simple 2D point class
struct Point {
    float x, y;

    Point(float xx = 0, float yy = 0) : x(xx), y(yy) {}

    Point operator+(const Point& o) const { return Point(x + o.x, y + o.y); }
    Point operator-(const Point& o) const { return Point(x - o.x, y - o.y); }
    Point operator*(float scalar) const { return Point(x * scalar, y * scalar); }
};



// 2D Camera
class Cam2D {
    public:
        Cam2D(Point pos_, float height_, int w, int h, float f_len_, float zoom_fac_);

        // mouse_pos.y is inverted as in python code (y *= -1)
        void update(const Point& mouse_pos_raw, bool mouse_pressed, int scroll_event);

        // Projects world point to screen pixel coords
        SDL_Point project_point(const Point& point);

        float query_scale_fac();
        SDL_Point query_window_dim();

    private:
        Point pos;
        float height;
        int window_width, window_height;
        float f_len;
        float zoom_fac;

        bool prev_press = false;
        Point pan_init_m;
        Point pan_init_pos;

        float scale_fac;
};



// Base Object class
class Object {
    public:
        Object(Point pos_, double rot, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_);

        virtual ~Object();

        virtual void render();

        void set_pos(Point pos);
        void set_angle(double theta);

    protected:
        Point pos;
        double rot;
        Cam2D* cam;
        SDL_Renderer* renderer;
        bool tag_en;
};

//Grid object
class Grid : public Object {
    public:
        Grid(Point pos_, float cell_size_, Cam2D* cam_, SDL_Renderer* rend_);
        void render() override;

    private:
        float cell_size;
};


// Line object
class Line : public Object {
    public:
        Line(Point start, Point end, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_);
        void render() override;

    private:
        Point end_pos;
};



// Circle object
class Circle : public Object {
    public:
        Circle(Point pos_, float size_, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_);

        void render() override;

    private:
        float size;
};




// Rect object
class Rect : public Object {
    public:
        Rect(Point pos_, float rot_, Point size_, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_);
        void render() override;

    private:
        Point size;
};


#endif