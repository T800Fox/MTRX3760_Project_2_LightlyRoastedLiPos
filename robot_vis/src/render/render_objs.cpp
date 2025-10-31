#include "render_objs.hpp"



Cam2D::Cam2D(Point pos_, float height_, int w, int h, float f_len_, float zoom_fac_) :
        pos(pos_), height(height_), window_width(w), window_height(h), f_len(f_len_), zoom_fac(zoom_fac_), scale_fac(0.0) {}


void Cam2D::update(const Point& mouse_pos_raw, bool mouse_pressed, int scroll_event) {
    Point mouse_pos = mouse_pos_raw;
    mouse_pos.y *= -1;

    if (mouse_pressed) {
        if (!prev_press) {
            pan_init_m = mouse_pos;
            pan_init_pos = pos;
        }
        pos = (pan_init_m - mouse_pos) * height + pan_init_pos;
    }
    prev_press = mouse_pressed;

    height = std::max(height + zoom_fac * -scroll_event, 0.5f);
    scale_fac = f_len / height;
}

SDL_Point Cam2D::project_point(const Point& point) {
    float cx = window_width / 2.0f;
    float cy = window_height / 2.0f;

    float dx = point.x - pos.x;
    float dy = point.y - pos.y;

    int screen_x = static_cast<int>(cx + dx * scale_fac);
    int screen_y = static_cast<int>(cy - dy * scale_fac);  // invert Y for screen

    return { screen_x, screen_y };
}

float Cam2D::query_scale_fac(){
    return scale_fac;
}

SDL_Point Cam2D::query_window_dim(){
    return { window_width, window_height};
}


//Do parameter _ for all ***
Object::Object(Point pos_, double rot_, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_)
        : pos(pos_), rot(rot_), cam(cam_), renderer(rend_), tag_en(tag_en_) {}

Object::~Object() {}

void Object::render(){}

void Object::set_pos(Point pos_){
    pos = pos_;
}

void Object::set_angle(double theta){
    rot = theta;
}


Grid::Grid(Point pos_, float cell_size_, Cam2D* cam_, SDL_Renderer* rend_) :
            Object(pos_, 0.0f, cam_, rend_, false), cell_size(cell_size_) {}


void Grid::render(){
    SDL_Point proj = cam->project_point(pos);

    float scale_fac = cam->query_scale_fac();

    int proj_size = (int) cell_size * scale_fac;
    int start_x = proj.x % proj_size;
    int start_y = proj.y % proj_size;

    SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);

    SDL_Point wind_dim = cam->query_window_dim();
    //Render vertical
    for (int i=0; i < wind_dim.x/proj_size + 1; i++){
        SDL_RenderDrawLine(renderer, start_x, 0, start_x, wind_dim.y);
        start_x += proj_size;
    }
    //Render horizontal
    for (int i=0; i < wind_dim.y/proj_size + 1; i++){
        SDL_RenderDrawLine(renderer, 0, start_y, wind_dim.x, start_y);
        start_y += proj_size;
    }
    
}


Line::Line(Point start, Point end, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_) :
        Object(start, 0.0f, cam_, rend_, tag_en_), end_pos(end) {}


void Line::render() {
    SDL_Point p1 = cam->project_point(pos);
    SDL_Point p2 = cam->project_point(end_pos);

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderDrawLine(renderer, p1.x, p1.y, p2.x, p2.y);

    Object::render();
}


Circle::Circle(Point pos_, float size_, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_) :
        Object(pos_, 0.0, cam_, rend_, tag_en_), size(size_) {}


void Circle::render() {
    // Draw circle using midpoint algorithm approx
    SDL_Point center = cam->project_point(pos);
    int radius = static_cast<int>(size * cam->query_scale_fac());

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);

    // Simple circle drawing
    for (int w = 0; w < radius * 2; w++) {
        for (int h = 0; h < radius * 2; h++) {
            int dx = radius - w;
            int dy = radius - h;
            if ((dx * dx + dy * dy) <= (radius * radius)) {
                SDL_RenderDrawPoint(renderer, center.x + dx, center.y + dy);
            }
        }
    }

    Object::render();
}



Rect::Rect(Point pos_, float rot_, Point size_, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_) :
        Object(pos_, rot, cam_, rend_, tag_en_), size(size_) {}


void Rect::render() {
    // Create rect at center projected point
    SDL_Point center = cam->project_point(pos);

    int w = static_cast<int>(size.x * cam->query_scale_fac());
    int h = static_cast<int>(size.y * cam->query_scale_fac());

    SDL_Rect rect{ center.x - w / 2, center.y - h / 2, w, h };

    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);

    // For rotation, SDL_RenderCopyEx could be used with texture, but we don't have a texture here
    // We'll draw rotated rect points manually

    // Calculate corners
    float rad = rot * M_PI / 180.0f;
    float cosr = std::cos(rad);
    float sinr = std::sin(rad);

    SDL_Point corners[4];
    // Local corners relative to center
    Point local_corners[4] = {
        { -w/2.0f, -h/2.0f },
        {  w/2.0f, -h/2.0f },
        {  w/2.0f,  h/2.0f },
        { -w/2.0f,  h/2.0f }
    };

    for (int i = 0; i < 4; ++i) {
        float x = local_corners[i].x;
        float y = local_corners[i].y;
        // rotate
        float rx = x * cosr - y * sinr;
        float ry = x * sinr + y * cosr;
        corners[i] = { static_cast<int>(center.x + rx), static_cast<int>(center.y + ry) };
    }

    // Draw edges
    SDL_RenderDrawLine(renderer, corners[0].x, corners[0].y, corners[1].x, corners[1].y);
    SDL_RenderDrawLine(renderer, corners[1].x, corners[1].y, corners[2].x, corners[2].y);
    SDL_RenderDrawLine(renderer, corners[2].x, corners[2].y, corners[3].x, corners[3].y);
    SDL_RenderDrawLine(renderer, corners[3].x, corners[3].y, corners[0].x, corners[0].y);

    Object::render();
}

