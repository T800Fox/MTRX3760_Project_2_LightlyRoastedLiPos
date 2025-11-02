#ifndef ROBOT_VIS_RENDER_OBJS_HPP_
#define ROBOT_VIS_RENDER_OBJS_HPP_

#include <SDL.h>
#include <SDL_ttf.h>
#include <cmath>
#include <string>
#include <iostream>
#include <cstdio>
#include <vector>

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
    bool operator==(const Point& other) const {return x == other.x && y == other.y;}

    bool operator!=(const Point& other) const {return !(*this == other);}
};


float point_distance(const Point & p1, const Point & p2);


// 2D Camera
class Cam2D {
    public:
        Cam2D() = default;
        Cam2D(Point pos_, float height_, int w, int h, float f_len_, float zoom_fac_);

        // mouse_pos.y is inverted as in python code (y *= -1)
        void update(const Point& mouse_pos_raw, bool mouse_pressed, int scroll_event);

        // Projects world point to screen pixel coords
        SDL_Point project_point(const Point& point);

        float query_scale_fac();
        SDL_Point query_window_dim();
        bool query_zoom_update();

        void override_pos(Point pos_);

    private:
        Point pos;
        float height;
        int window_width, window_height;
        float f_len;
        float zoom_fac;
        bool zoom_update_flag; // Flag for updating text size (requires reload)

        bool prev_press = false;
        Point pan_init_m;
        Point pan_init_pos;

        float scale_fac;
};



// Base Object class
class Object {
    public:
        Object() = default;
        Object(Point pos_, double rot, SDL_Colour colour_, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_);

        virtual ~Object();

        virtual void render();

        void set_tag_en(bool tag_en_);
        void set_pos(Point pos);
        void set_angle(double theta);

        Point query_pos();
        double query_rot();

    protected:
        Point pos;
        double rot;
        Cam2D* cam;
        SDL_Renderer* renderer;
        bool tag_en;

        int font_size;
        TTF_Font* font;

        SDL_Colour colour;

        //Methods
        void init_font();
        void set_colour();
};

//Grid object
class Grid : public Object {
    public:
        Grid() = default;
        Grid(Point pos_, float cell_size_, bool *occupancy_enabled_, Cam2D* cam_, SDL_Renderer* rend_);
        void render() override;

        void link_cell_data(std::vector<char> &cell_data_, int grid_width_, int grid_height_, Point origin_offset_);

    private:
        float cell_size;

        bool *occupancy_enabled;
        int grid_width, grid_height;
        std::vector<char> cell_data;
};


// Line object
class Line : public Object {
    public:
        Line() = default;
        Line(Point start, Point end, SDL_Colour colour_, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_);
        void render() override;
        Point query_end_pos();

    private:
        Point end_pos;
};



// Circle object
class Circle : public Object {
    public:
        Circle() = default;
        Circle(Point pos_, float size_, SDL_Colour colour_, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_);

        void render() override;

    private:
        float size;
};




// Rect object
class Rect : public Object {
    public:
        Rect() = default;
        Rect(Point pos_, float rot_, Point size_, SDL_Colour colour_, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_);
        void render() override;
        

    private:
        Point size;
};




#endif