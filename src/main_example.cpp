// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: main_example.cpp
// Brief: Minimal example showing how to call PlanningCore from a delivery node.

#include <iostream>
#include <vector>
#include "planning_core.h"

int main() {
    // 1) Create planner (turn penalty in meters per heading change)
    PlanningCore planner(0.10);

    // 2) Fill SLAM occupancy (ROS-style 0..100, -1 unknown), with rows/cols/resolution
    // Replace these with real data from your SLAM/OccupancyGrid
    int rows = 100;
    int cols = 100;
    double resolution_m = 0.05;
    std::vector<int8_t> occ100(rows * cols, 0); // all free for example

    PlanningCore::SlamMap100 slam;
    slam.occ100 = std::move(occ100);
    slam.rows = rows;
    slam.cols = cols;
    slam.resolution_m = resolution_m;
    slam.occ_thresh100 = 65;         // >= occupied
    slam.unknown_as_occupied = true; // -1 treated as occupied (safer)

    if (!planner.linkMap(slam)) {
        std::cerr << "Failed to link map\n";
        return 1;
    }

    // Optional obstacle inflation (meters). Set 0.0 to disable.
    planner.inflate(0.0);

    // 3) Set start (grid cell coordinates)
    planner.setStartCell(10, 10);

    // 4) Provide item targets in world millimeters from bottom-right origin
    std::vector<PlanningCore::ItemTarget> items = {
        {"ItemA", 300, 1800, 'A', 2},
        {"ItemB", 400,  900, 'B', 2},
        {"ItemC", 300,  100, 'C', 1}
    };
    planner.setItems(items);

    // 5) Plan route
    std::vector<GridMap::Cell> fullPath;
    std::vector<GroupReport> reports;
    if (!planner.plan(fullPath, reports)) {
        std::cerr << "Route infeasible\n";
        return 2;
    }

    // 6) Build navigation commands (0=Rotate, 1=Translate)
    auto cmds = planner.buildCommands(/*fixedInitialHeading=*/std::nullopt);

    // 7) Convert to [type,value] pairs (like planning_results.csv) and print
    for (const auto& c : cmds) {
        int t = (c.type == CmdType::Rotate) ? 0 : 1;
        std::cout << t << "," << c.value << "\n";
    }

    return 0;
}


