// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: main.cpp
// Author: Lightly Roasted Lipos
// Description: Driver — loads map, solves priority groups using polymorphic planner,
//              prints reports, ASCII, and nav command stream.

#include <iostream>
#include <fstream>
#include <iomanip>
#include <array>
#include <map>

#include "grid_map.h"
#include "a_star_planner.h"
#include "route_solver.h"
#include "commands.h"
#include "viz.h"


int main() {

    std::ofstream log("planning_results.csv");
    if (!log.is_open()) { std::cerr << "Cannot open planning_results.txt\n"; return 1; }

    GridMap map(0.02);
    // if (!map.loadCSV("maze_grid.csv", /*rows*/30, /*cols*/59)) {
    if (!map.loadCSV("out.csv", /*rows*/101, /*cols*/105)) {
        std::cerr << "Map load failed\n"; return 1;
    }

    // Start (grid coords)
    GridMap::Cell start{10, 10};


    // Items (mm from bottom-right), labels, priorities
    std::vector<Item> items = {
        //maze_grid.csv items
        // {"ItemA", 2400, 1400, 'A', 2},
        // {"ItemB", 2800, 1100, 'B', 2},
        // {"ItemC", 1700, 1350, 'C', 1},
        // {"ItemD", 50, 1000, 'D', 1}

        // out.csv items
        // {"ItemA", 800, 2000, 'A', 2},
        // {"ItemB", 800, 800, 'B', 2},
        // {"ItemC", 2000, 400, 'C', 1},
        // {"ItemD", 600, 4500, 'D', 1}
        
        {"ItemA", 300, 1800, 'A', 2},
        {"ItemB", 400, 900, 'B', 2},
        {"ItemC", 300, 100, 'C', 1},
        {"ItemD", 600, 1500, 'D', 1}
    };

    // Polymorphic planner — can swap to other planners via PathPlanner interface
    AStarPlanner astar;
    // Turning Parameter
    astar.setTurnWeight(0.1); // penalize turns by X m each
    RouteSolver solver(map, astar);

    std::vector<GridMap::Cell> fullPath;
    std::vector<GroupReport> reports;

    if (!solver.solvePriorityGroups(start, items, fullPath, reports)) {
        std::cerr << "Route infeasible for one or more priority groups\n"; return 2;
    }

    std::cout << std::fixed << std::setprecision(3);
    log      << std::fixed << std::setprecision(3);

    // Reports
    double grandTotal = 0.0;
    for (const auto& rep : reports) {
        std::cout << "=== Priority " << rep.priority << " group ===\n";
        // log      << "=== Priority " << rep.priority << " group ===\n";
        std::cout << "  Order: ";
        // log      << "  Order: ";
        for (size_t i = 0; i < rep.orderNodeIds.size(); ++i) {
            int id = rep.orderNodeIds[i];
            if (id == 0) { std::cout << "Start"; /*log << "Start";*/ }
            else { std::cout << items[id-1].name; /*log << items[id-1].name;*/}
            if (i + 1 < rep.orderNodeIds.size()) { std::cout << " -> "; /*log << " -> ";*/ }
        }
        std::cout << "\n  Group distance (inc. return): " << rep.total_m << " m\n\n";
        // log      << "\n  Group distance (inc. return): " << rep.total_m << " m\n\n";
        grandTotal += rep.total_m;
    }
    std::cout << "TOTAL traversal distance: " << grandTotal << " m\n\n";
    // log      << "TOTAL traversal distance: " << grandTotal << " m\n\n";

    // Markers for ASCII
    std::vector<std::pair<char, GridMap::Cell>> markers;
    markers.emplace_back('S', start);
    for (const auto& it : items)
        markers.emplace_back(it.label, map.worldMmToCell(it.x_mm, it.y_mm));

    std::cout << "ASCII map of FULL traversal:\n";
    // std::cout << "Map with Turning Cost = 0.25:\n";
    printAsciiFull(map, fullPath, markers);

    // Build nav commands (Rotate: 0=R,1=L,2=U,3=S; Translate: meters)
    auto cmds = buildCommandsFromPath(map, fullPath /*, fixedInitialHeading = std::nullopt*/);

    std::cout << "\nNAV COMMANDS (Rotate=0 TRANSLATE=1, 0=right 1=left 2=U-Turn X=distance in m):\n";
    // log      << "\nNAV COMMANDS (Rotate=0 TRANSLATE=1, 0=right 1=left 2=U-Turn X=distance in m):\n";

    double move_sum = 0.0;
    for (const auto& c : cmds) {
        if (c.type == CmdType::Rotate) {
            int dir = static_cast<int>(c.value);
            std::cout << "ROTATE " << dir << "\n";
            log      << "0," << dir << "\n";
        } else {
            move_sum += c.value;
            std::cout << "TRANSLATE " << c.value << "\n";
            log      << "1," << c.value << "\n";
        }
    }
    std::cout << "Sum of TRANSLATE distances: " << move_sum << " m\n";
    // log      << "Sum of TRANSLATE distances: " << move_sum << " m\n";

    std::cout << "\nResults also saved to planning_results.txt\n";
    return 0;
}
