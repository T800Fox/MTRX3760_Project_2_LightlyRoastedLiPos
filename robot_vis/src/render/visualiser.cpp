


//Delivery request
struct PackRequest {
    int IDindex;
    int priority;
};





Visualiser::Visualiser(SDL_Window *window, SDL_Renderer *renderer) 
        : renderer(renderer), window(window), is_menu_open(true), camera_track_en(false), 
        robot_state(AWAIT_REQUEST), m_state(LOGIC), has_been_inspected(true){

    // Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    packages[1023] = Package{Point{0.0,0.0}};
    packages[5297] = Package{Point{0.0,0.0}};
    packages[2763] = Package{Point{0.0,0.0}};
    packages[2362] = Package{Point{0.0,0.0}};


    Cam2D cam(Point(0, 0), 1.0f, WINDOW_WIDTH, WINDOW_HEIGHT, 1.0f, 0.2f);

    // Create objects
    Line line(Point(-1, -1), Point(2, 2), &cam, renderer, true);
    Circle circle(Point(1, 1), 30, &cam, renderer, true);
    Rect rect(Point(0, 0), Point(100, 50), 45.0f, &cam, renderer, true);



    

}

Visualiser::~Visualiser(){}


void Visualiser::package_detected_callback(){

    int key = 1029;
    double pos_x = 0.0;
    double pos_y = 0.0;

    packages[key] = Package{ Point{pos_x,pos_y} };
    
}


void Visualiser::render(){
    //Render actual map ----

    //Render UI (and allow ImGUI to update relevant state variables)
    render_UI(); 

}



void Visualiser::render_UI(){
    //New frame
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    ImVec2 display = ImGui::GetIO().DisplaySize;

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
                robot_state = INSPECT_WAREHOUSE;
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

            if (robot_state == INSPECT_WAREHOUSE){
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

            DisplayPackageSelection();

            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.9f, 1.0f));       // normal
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.7f, 1.0f, 1.0f)); // hover
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.5f, 0.8f, 1.0f));  // pressed

            ImGui::Dummy(ImVec2(0.0f, 5.0f)); // Vertical separator
            if (ImGui::Button("Request Delivery", ImVec2(140, 20))) { // size in pixels
                
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
            ImGui::Checkbox("Camera track", &camera_track_en);
            //ImGui::SliderFloat("Timestep", &dt, 0.001f, 0.1f);
            if (ImGui::Button("Reset")) { }
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


void Visualiser::DisplayPackageSelection(){
    static std::vector<PackRequest> pack_requests = { };

    //Convert package IDs to vector of strings
    std::vector<std::string> idStrings;
    std::vector<const char*> idOptions;
    idStrings.reserve(packages.size());
    for (const auto &[key, v] : packages){
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