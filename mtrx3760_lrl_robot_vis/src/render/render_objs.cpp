#include "mtrx3760_lrl_robot_vis/render_objs.hpp"


float point_distance(const Point & p1, const Point & p2){
    return sqrt((p1.x-p2.x) * (p1.x-p2.x) + (p1.y-p2.y) * (p1.y-p2.y));
}


Cam2D::Cam2D(Point pos_, float height_, int w, int h, float f_len_, float zoom_fac_) :
        pos(pos_), height(height_), window_width(w), window_height(h), f_len(f_len_), zoom_fac(zoom_fac_), scale_fac(f_len_/height) {}


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

    zoom_update_flag = (scroll_event != 0);
}

SDL_Point Cam2D::project_point(const Point& point) {
    float cx = window_width / 2.0f;
    float cy = window_height / 2.0f;

    float dx = point.x - pos.x;
    float dy = point.y - pos.y;

    int screen_x = (int)(cx + dx * scale_fac);
    int screen_y = (int)(cy - dy * scale_fac);  // invert Y for screen

    return { screen_x, screen_y };
}

void Cam2D::override_pos(Point pos_){
    pos = pos_;
}

float Cam2D::query_scale_fac(){
    return scale_fac;
}

SDL_Point Cam2D::query_window_dim(){
    return { window_width, window_height};
}

bool Cam2D::query_zoom_update(){
    return zoom_update_flag;
}


//Do parameter _ for all ***
Object::Object(Point pos_, double rot_, SDL_Colour colour_, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_)
        : pos(pos_), rot(rot_), colour(colour_), cam(cam_), renderer(rend_), tag_en(tag_en_), font_size(30) {
    
    init_font(); //Load font with projected size
}

Object::~Object() {}


Point Object::query_pos(){
    return pos;
}

double Object::query_rot(){
    return rot;
}

void Object::set_pos(Point pos_){
    pos = pos_;
}

void Object::set_angle(double theta){
    rot = theta;
}

void Object::set_tag_en(bool tag_en_){
    tag_en = tag_en_;
}

//Set SDL draw colour
void Object::set_colour(){
    SDL_SetRenderDrawColor(renderer, colour.r, colour.g, colour.b, 255);
}

void Object::init_font(){
    int size = (int) (cam->query_scale_fac() * font_size);
    font = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", size);
    std::cout << size;
}

void Object::render(){
    if (tag_en){
        //Check if zoom update occured (scroll event)
        if (cam->query_zoom_update()){
            init_font();
        }

        //Generate string
        char buf[64];
        std::snprintf(buf, sizeof(buf), "(x: %.2f, y: %.2f, rot: %.2f)", pos.x, pos.y, rot);
        
        //Project pos REPLACE HARDCODED OFFSET
        SDL_Point projected_pos = cam->project_point(pos + Point{140,-140});

        //Generate surface
        SDL_Surface* textSurface = TTF_RenderText_Solid(font, buf, {255, 255, 255, 255});
        SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
        SDL_FreeSurface(textSurface);
        SDL_Rect textRect = {projected_pos.x, projected_pos.y, 0, 0};
        SDL_QueryTexture(textTexture, nullptr, nullptr, &textRect.w, &textRect.h);

        SDL_RenderCopy(renderer, textTexture, nullptr, &textRect); //Blit
        SDL_DestroyTexture(textTexture); //Cleanup
    }

}



Grid::Grid(Point pos_, float cell_size_, bool *occupancy_enabled_, Cam2D* cam_, SDL_Renderer* rend_) :
            Object(pos_, 0.0f, SDL_Colour{0,0,0,0}, cam_, rend_, false), cell_size(cell_size_), grid_width(0), grid_height(0), occupancy_enabled(occupancy_enabled_){}


void Grid::link_cell_data(std::vector<char> &cell_data_, int grid_width_, int grid_height_, Point origin_offset_){
    cell_data = cell_data_;
    pos = origin_offset_;
    grid_width = grid_width_;
    grid_height = grid_height_;
}



void Grid::render(){
    float scale_fac = cam->query_scale_fac();

    float proj_size = cell_size * scale_fac;
    SDL_Point proj = cam->project_point(pos);

    float start_x = std::fmod(proj.x, proj_size);
    float start_y = std::fmod(proj.y, proj_size);
    

    //Render cell-borders
    SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);

    SDL_Point wind_dim = cam->query_window_dim();
    //Render vertical
    for (int i=0; i < wind_dim.x / (int)proj_size + 1; i++){
        SDL_RenderDrawLine(renderer, (int) start_x, 0, (int) start_x, wind_dim.y);
        start_x += proj_size;
    }
    //Render horizontal
    for (int i=0; i < wind_dim.y / (int)proj_size + 1; i++){
        SDL_RenderDrawLine(renderer, 0, (int) start_y, (int) wind_dim.x, start_y);
        start_y += proj_size;
    }


    if (*occupancy_enabled){
        //Render cell-occupancy
        SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);

        SDL_Rect temp_rect;
        for (int y = 0; y < grid_height; y++){
            for (int x = 0; x < grid_width; x++){
                //Fill cell if occupied
                if (cell_data[x + y*grid_width]){
                    temp_rect = SDL_Rect{proj.x + (int) (proj_size*x), proj.y - (int) (proj_size*y), 
                                                        (int) proj_size, (int) proj_size};

                    SDL_RenderFillRect(renderer, &temp_rect);
                }
            }
        }
    }
}


Line::Line(Point start, Point end, SDL_Colour colour_, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_) :
        Object(start, 0.0f, colour_, cam_, rend_, tag_en_), end_pos(end) {}


void Line::render() {
    set_colour();

    SDL_Point p1 = cam->project_point(pos);
    SDL_Point p2 = cam->project_point(end_pos);

    SDL_RenderDrawLine(renderer, p1.x, p1.y, p2.x, p2.y);

    Object::render();
}

Point Line::query_end_pos(){
    return end_pos;
}


Circle::Circle(Point pos_, float size_, SDL_Colour colour_, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_) :
        Object(pos_, 0.0, colour_, cam_, rend_, tag_en_), size(size_) {}


void Circle::render() {
    // Draw circle using midpoint algorithm approx
    SDL_Point center = cam->project_point(pos);
    int radius = static_cast<int>(size * cam->query_scale_fac());

    set_colour();
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



Rect::Rect(Point pos_, float rot_, Point size_, SDL_Colour colour_, Cam2D* cam_, SDL_Renderer* rend_, bool tag_en_) :
        Object(pos_, rot, colour_, cam_, rend_, tag_en_), size(size_) {}


void Rect::render() {
    // Create rect at center projected point
    SDL_Point center = cam->project_point(pos);

    int w = static_cast<int>(size.x * cam->query_scale_fac());
    int h = static_cast<int>(size.y * cam->query_scale_fac());

    SDL_Rect rect{ center.x - w / 2, center.y - h / 2, w, h };

    set_colour();

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

