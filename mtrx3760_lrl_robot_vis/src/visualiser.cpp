#include "visualiser.hpp"





Visualiser::Visualiser(SDL_Window *window, SDL_Renderer *renderer) 
        : renderer(renderer), window(window), is_menu_open(true), camera_track_en(false), wall_follower_map_en(true), slam_map_en(false),
        robot_state(AWAITING_REQUEST), m_state(LOGIC), has_been_inspected(true), hover_pkg_id(-1){


    //Connect to socket
    std::thread([&](){
        socket.connect_to_server("127.0.0.1", 8080);
    }).detach();
    


    // Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);


    //Active cam
    int WINDOW_WIDTH, WINDOW_HEIGHT;
    SDL_GetWindowSize(window, &WINDOW_WIDTH, &WINDOW_HEIGHT);
    
    //Active cam
    cam = Cam2D(Point{0, 0}, 1.0f, WINDOW_WIDTH, WINDOW_HEIGHT, 1.0f, 0.2f);

    //Gird 20mm
    grid = Grid(Point{0,0}, 20.0f, &slam_map_en, &cam, renderer);
    
    //Origin marker
    origin = Circle(Point{0,0}, 5.0f, WHITE, &cam, renderer, false);

    // Robot components
    robot_body = Rect(Point{0, 0}, 10.0f, Point{138, 178}, GREEN, &cam, renderer, true); //256mm square body
    robot_marker = Rect(Point{0, 0}, 10.0f, Point{20, 80}, GREEN, &cam, renderer, false); //Heading marker

}

Visualiser::~Visualiser(){
    SDL_DestroyTexture(texture);
}



void Visualiser::handle_ui_request(){

    switch (last_request){
        case NONE:{
            break;
        }
        case INSPECT_WAREHOUSE:{
            //Send TCP packet to interface node requesting warehouse-inspection
            json j = json{
                {"type", "srv_inspect_warehouse"}
            };
            socket.send_json(j);

            std::cout << "Inecpstr!!!!!";

            break;
        }
        case PERFORM_DELIVERY:{
            //Send TCP packet to interface node requesting delivery (package requests and solver params)
            json j = json{
                {"type", "srv_perform_delivery"},
                {"package_requests", pack_requests},
                {"solver_params", solver_params}
            };
            socket.send_json(j);

            std::cout << "delivery!!!!!";
            break;
        }
    }
}




void Visualiser::update(Point mouse_pos, bool mouse_pressed, int scroll_event){
    //Handle request from last ui update/render
    handle_ui_request();

    //Determine if a package is getting hovered on (info preview)
    hover_pkg_id = -1;
    SDL_Point screen_point;
    int ID_index = 0;
    for (const auto [key,value] : detected_packages){
        screen_point = cam.project_point(value.global_pos);

        //Circle point collision
        if (point_distance(Point{screen_point.x, screen_point.y}, mouse_pos) < 15.0f){
            //Mouse is hovering over package - break
            hover_pkg_id = key;
            
            //If clicked add to list of package requests (shorthand)
            if (mouse_pressed && !prev_press){
                pack_requests.push_back(PackRequest{ID_index, 0});
            }
            break;
        }

        ID_index++;
    }

    prev_press = mouse_pressed;




    
    json image = socket.query_prev_packet("data_image", true);
    try {
        if (!image.empty()){

            const std::string& encoded = image["image_data"];
            std::vector<unsigned char> raw_bytes(encoded.size());
            int decoded_len = hv_base64_decode(encoded.c_str(), encoded.size(), raw_bytes.data());
            raw_bytes.resize(decoded_len);


            SDL_Surface* surface = SDL_CreateRGBSurfaceFrom(
                raw_bytes.data(),
                image["width"].get<int>(),
                image["height"].get<int>(),
                24,                               // bits per pixel
                image["step"].get<int>(),                        // bytes per row
                0x000000FF, 0x0000FF00, 0x00FF0000, 0
            );

            texture = SDL_CreateTextureFromSurface(renderer, surface);
            SDL_FreeSurface(surface);
        }

    } catch (const std::exception &e) {
        // This catches json::parse_error, out_of_range, type_error, etc.
        std::cerr << "[WARN] Dropped bad pose packet: " << e.what() << "\n";
    }



    //Vis stuff
    //Parse data from packet and update render objects
    json pose = socket.query_prev_packet("data_pose");
    try {
        if (!pose.empty()){
            float x = pose["x"].get<float>();
            float y = pose["y"].get<float>();
            float rot = pose["rot"].get<float>();

            Point temp_pos = Point{x,y};

            robot_body.set_pos(temp_pos);
            robot_body.set_angle(rot);

            double proj_rot = (rot+90.0) * 3.14159/180.0; //Angle in rad to project marker position (forwards heading)
            robot_marker.set_pos(temp_pos + Point{sin(proj_rot) * 138/2, cos(proj_rot) * 138/2});
            robot_marker.set_angle(rot);

            //Add new point in path - only if update has occured (difference of 10mm<)
            Point prev_end_point = recent_path.empty() ? Point{0,0} : recent_path.back().query_end_pos();
            if (recent_path.empty() || point_distance(prev_end_point, temp_pos) > 20.0f) {
                recent_path.push_back(Line(prev_end_point, temp_pos, RED, &cam, renderer, false));
                if (recent_path.size() >= 1000)
                    recent_path.pop_front();
            }
        }

    } catch (const std::exception &e) {
        // This catches json::parse_error, out_of_range, type_error, etc.
        std::cerr << "[WARN] Dropped bad pose packet: " << e.what() << "\n";
    }


    //Map query
    json map = socket.query_prev_packet("data_map");
    try {
        if (!map.empty()){
            map_data = map["data"].get<std::vector<char>>();
            //Map recieved - parse and link grid objects cell-data
            grid.link_cell_data(map_data, map["width"].get<int>(), map["height"].get<int>(),
                            Point{map["origin_x"].get<float>(), map["origin_y"].get<float>()});
        }

    } catch (const std::exception &e) {
        // This catches json::parse_error, out_of_range, type_error, etc.
        std::cerr << "[WARN] Dropped bad map packet: " << e.what() << "\n";
    }


    //Package query (and clear packet)
    json package = socket.query_prev_packet("data_package_detection", true);
    try {
        if (!package.empty()){
            //Populate package struct with data from json
            Package package_struct;
            package_struct.ID = package["id"].get<int>();
            package_struct.global_pos = Point{package["pos_x"].get<float>(), package["pos_y"].get<float>()};
            package_struct.confidence = package["var"].get<double>();
            package_struct.observation_count = package["observation_count"].get<int>();

            detected_packages[package_struct.ID] = package_struct; //Umap entry
        }

    } catch (const std::exception &e) {
        // This catches json::parse_error, out_of_range, type_error, etc.
        std::cerr << "[WARN] Dropped bad package packet: " << e.what() << "\n";
    }


    json line_seg = socket.query_prev_packet("data_wall_follower_seg", true);
    try {
        if (!line_seg.empty()){
            //Create new line object (as wall-segment)
            wall_follower_line_segs.push_back(Line(Point{line_seg["start_pos_x"].get<float>(), line_seg["start_pos_y"].get<float>()}, 
                                                    Point{line_seg["end_pos_x"].get<float>(), line_seg["end_pos_y"].get<float>()}, 
                                                    PINK, &cam, renderer, false));
        }

    } catch (const std::exception &e) {
        // This catches json::parse_error, out_of_range, type_error, etc.
        std::cerr << "[WARN] Dropped bad map packet: " << e.what() << "\n";
    }
    


    //Amount of x-space occupied by menu (void mouse press events for visualiser if in menu)
    int menu_zone = is_menu_open ? cam.query_window_dim().x/4 : 0;

    mouse_pressed &= (menu_zone < mouse_pos.x);
    cam.update(mouse_pos, mouse_pressed, scroll_event);

    robot_body.set_tag_en(pose_tag_en);

    if (camera_track_en){
        cam.override_pos(robot_body.query_pos());
    }
}

void Visualiser::render(){
    //Render actual map ----
    grid.render();
    origin.render();


    //Render wall-follower map (if enabled)
    if (wall_follower_map_en){
        for (auto &seg : wall_follower_line_segs){
            seg.render();
        }
    }

    //Render robot path (if enabled)
    if (robot_path_en){
        for (Line& l : recent_path){
            l.render();
        }
    }

    //Render package tags
    Circle circle;
    for(const auto [key, value] : detected_packages){
        circle = Circle(value.global_pos, 20, BLUE, &cam, renderer, false);
        circle.render();
    }

    robot_body.render();
    robot_marker.render();


    //Render UI (and allow ImGUI to update relevant state variables)
    render_UI(); 
}



void Visualiser::render_UI(){
    //New frame
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    //Render info popup for hovered package
    render_package_data();

    ImVec2 display = ImGui::GetIO().DisplaySize;

    last_request = NONE; //Default request

    // --- Left side panel ---
    if (is_menu_open) {
        // Fix position & size: full height, 1/4 width
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(display.x * 0.25f, display.y));

        ImGui::Begin("Side Panel", &is_menu_open,
                    ImGuiWindowFlags_NoCollapse |
                    ImGuiWindowFlags_NoResize |
                    ImGuiWindowFlags_NoMove |
                    ImGuiWindowFlags_NoTitleBar);

        // Remove padding so it fills tight
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 8));
        

        //Interface for sending commands to robot
        if (ImGui::CollapsingHeader("Robot interface", ImGuiTreeNodeFlags_DefaultOpen)) {

            ImGui::Text("Inspect Warehouse");
            ImGui::Separator();

            //Main state-controller
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.9f, 1.0f));       // normal
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.7f, 1.0f, 1.0f)); // hover
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.5f, 0.8f, 1.0f));  // pressed

            if (ImGui::Button("Inspect", ImVec2(60, 20))) { // size in pixels
                //Inspect
                last_request = INSPECT_WAREHOUSE;
            }
            ImGui::PopStyleColor(3);

            if (!has_been_inspected){
                //Indicate that warehouse hasn't yet been inspected
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2, 2));
                ImGui::SetWindowFontScale(0.9f);  // 80% of normal size
                ImGui::Text("* Warehouse hasn't been inspected *");
                ImGui::SetWindowFontScale(1.0f);
                ImGui::PopStyleVar();
            }

            if (robot_state == INSPECTING){
                ImGui::ProgressBar(-1.0f * (float)ImGui::GetTime(), ImVec2(0.0f, 0.0f), "Inspecting..");
                //ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
            }


            //Disable delivery functionality till an inspection has been complete
            if (!has_been_inspected){
                ImGui::BeginDisabled();
            }


            ImGui::Dummy(ImVec2(0.0f, 20.0f)); // Vertical separator
            ImGui::Text("Deliver Packages");
            ImGui::Separator();
            //Solver settings
            ImGui::Text("Solver parameters:");
            ImGui::SliderFloat("Turn cost", &solver_params.turn_cost, 0.0f, 1.0f);


            //Selection
            ImGui::Dummy(ImVec2(0.0f, 20.0f)); // Vertical separator
            ImGui::Text("Package selection:");
            display_package_selection();

            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.9f, 1.0f));       // normal
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.7f, 1.0f, 1.0f)); // hover
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.5f, 0.8f, 1.0f));  // pressed

            ImGui::Dummy(ImVec2(0.0f, 5.0f)); // Vertical separator
            if (ImGui::Button("Request Delivery", ImVec2(140, 20))) { // size in pixels
                last_request = PERFORM_DELIVERY;
            }
            ImGui::PopStyleColor(3);


            //end disable
            if (!has_been_inspected){
                ImGui::EndDisabled(); 
            }
        
        }


        ImGui::Dummy(ImVec2(0.0f, 20.0f)); // Vertical separator
        //Settings 
        if (ImGui::CollapsingHeader("Visualiser Settings", ImGuiTreeNodeFlags_CollapsingHeader)) {
            ImGui::Text("General display:");
            ImGui::Checkbox("Display robot path", &robot_path_en);
            ImGui::Checkbox("Display pose tags", &pose_tag_en);

            ImGui::Dummy(ImVec2(0.0f, 10.0f)); // Vertical separator
            ImGui::Text("Map visibility:");
            ImGui::Checkbox("Display SLAM-occupancy", &slam_map_en);
            ImGui::Checkbox("Display wall-follower map", &wall_follower_map_en);

            ImGui::Dummy(ImVec2(0.0f, 10.0f)); // Vertical separator
            ImGui::Text("Misc:");
            ImGui::Checkbox("Camera track", &camera_track_en);

        }



        ImGui::PopStyleVar();
        ImGui::End();
    }


    // compute where the toggle should go
    float toggle_x = is_menu_open ? display.x * 0.25f -10.0f : -10.0f;
    float toggle_y = 10.0f;

    // Floating toggle button
    ImGui::SetNextWindowPos(ImVec2(toggle_x, toggle_y));
    ImGui::Begin("Toggle", nullptr,
                ImGuiWindowFlags_NoDecoration |
                ImGuiWindowFlags_AlwaysAutoResize |
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoSavedSettings |
                ImGuiWindowFlags_NoBackground);
    if (ImGui::Button(is_menu_open ? "<" : ">"))
        is_menu_open = !is_menu_open;

    ImGui::End();


    ImGui::Render();
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);

}


void Visualiser::render_package_data(){
    if (hover_pkg_id != -1) {
        auto& pkg = detected_packages[hover_pkg_id]; // get the package data
        SDL_Point screen_pos = cam.project_point(pkg.global_pos);

        // Offset slightly so popup isn't exactly on top of the point
        ImVec2 popup_pos = ImVec2(screen_pos.x + 10, screen_pos.y - 10);
        ImGui::SetNextWindowPos(popup_pos, ImGuiCond_Always);
        ImGui::Begin("Package Info", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar);

        float available_x = ImGui::GetContentRegionAvail().x;
        ImGui::Image(texture, ImVec2(available_x, available_x * 0.7));
        ImGui::Text("Package ID: %d", pkg.ID);
        ImGui::Text("Confidence: %.2f", pkg.confidence);
        ImGui::Text("Observations: %d", pkg.observation_count);
        ImGui::End();
    }
}

void Visualiser::display_package_selection(){
    //Convert package IDs to vector of strings
    std::vector<std::string> idStrings;
    std::vector<const char*> idOptions;
    idStrings.reserve(detected_packages.size());
    for (const auto &[key, v] : detected_packages){
        idStrings.push_back("#"+std::to_string(key));
        idOptions.push_back(idStrings.back().c_str());
    }


    for (int i = 0; i < pack_requests.size(); ++i)
    {
        ImGui::PushID(i); // unique ID for each row

        // === Row layout ===
        ImGui::SetNextItemWidth(120);
        ImGui::Combo("##id", &pack_requests[i].IDindex, idOptions.data(), idOptions.size());

        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ImGui::DragInt("##priority", &pack_requests[i].priority, 0.1, 0);

        ImGui::SameLine();
        if (ImGui::Button("-")) {
            pack_requests.erase(pack_requests.begin() + i);
            ImGui::PopID();
            break; // vector modified → break out
        }

        ImGui::PopID();
    }

    // Add button
    if (ImGui::Button("+ Add Task")) {
        pack_requests.push_back({0, 0});
    }

}