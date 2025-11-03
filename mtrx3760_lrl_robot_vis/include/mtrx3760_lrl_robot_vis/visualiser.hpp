#ifndef VIS_HEADER
#define VIS_HEADER

#include <SDL.h>
#include <vector>
#include <deque>
#include <unordered_map>
#include <string>
#include <iostream>
#include "mtrx3760_lrl_robot_vis/render_objs.hpp"
#include "imgui.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_sdlrenderer2.h"
#include "mtrx3760_lrl_robot_vis/socket.hpp"
#include "base64.h"


#define WHITE  SDL_Colour{255, 255, 255, 255}  
#define RED    SDL_Colour{255,  50,  50, 255}  
#define GREEN  SDL_Colour{ 50, 255,  50, 255} 
#define BLUE   SDL_Colour{ 50,  50, 255, 255}  
#define PINK   SDL_Colour{255, 100, 180, 255}   
#define BLACK  SDL_Colour{  0,   0,   0, 255}   
#define SKYBLUE SDL_Colour{ 50, 200, 255, 255}  


enum master_state{
    RECHARGE_BATTERY,
    RECENTER,
    LOGIC,
};

enum state{
    INSPECTING,
    AWAITING_REQUEST,
    DELIVERING,
};


//Request from ImGUI
enum RequestUI{
    NONE,
    INSPECT_WAREHOUSE,
    PERFORM_DELIVERY,
};


struct Package{
    int ID;
    Point global_pos;
    double confidence_radius;
    int observation_count;
    SDL_Texture* closest_image_tex;
};



//Delivery request (within UI)
struct PackRequest {
    int IDindex; //Don't serialise (for UI)
    uint32_t ID;
    int priority;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(PackRequest, ID, priority); //for json serialize
};


struct SolverParams {
    //Strings associated with solver types for UI selection
    static constexpr std::array<const char*, 2> ui_solver_types = {
        "A-star",
        "Theta-star (any angle)" // Use standard case/spelling
    };

    //Serialize ---
    float turn_cost;
    enum SolverType {
        A_STAR,
        THETA,
    } solver_type;
    
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(SolverParams, turn_cost, solver_type); 
};


class Visualiser{
    public:
        Visualiser(SDL_Window *window, SDL_Renderer *renderer);
        ~Visualiser();
        
        //Ros topic callbacks  HIGH QOS FOR SUBSCRIBER
        void package_detected_callback(/*msg with ID and pos*/);

        //Load data and update
        void update(Point mouse_pos, bool mouse_pressed, int scroll_event);

        //Render visualiser + UI
        void render();
        

    private:
        // Methods
        void render_UI();
        void render_package_data();
        void display_package_selection();

        void handle_ui_request();

        //TCP connection for recieving data
        TCP_Connection socket;

        //SDL2 references
        SDL_Window *window;
        SDL_Renderer *renderer;

        //UI state
        bool is_menu_open;
        std::vector<PackRequest> pack_requests;
        SolverParams solver_params{0.5f};
        RequestUI last_request; //Request state from last IMgui render/update
        bool camera_track_en;
        bool robot_path_en;
        bool pose_tag_en;
        bool slam_map_en;
        bool wall_follower_map_en;
        int hover_pkg_id; //ID of package popup selected
        bool prev_press; //Mouse pressed prev update (for click)

        //Robot state
        master_state m_state;
        state robot_state;
        std::deque<Line> recent_path; //List of segments in robot path
        std::vector<char> map_data; //Occupancy grid from /map

        bool has_been_inspected; // Flag for whether warehouse has been inspected (can delivery be performed)
        std::unordered_map<uint32_t, Package> detected_packages; // Map of packages detcted in inspection (int is ID)

        //Render objects -----------
        Rect robot_body;
        Rect robot_marker;
        Circle origin;
        Grid grid;
        Cam2D cam;
        std::vector<Line> wall_follower_line_segs;

        
        SDL_Texture* texture;


};

#endif
