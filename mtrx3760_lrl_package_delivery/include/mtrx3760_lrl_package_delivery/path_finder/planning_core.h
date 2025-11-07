// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: planning_core.h
// Author: Oliver Lennox
// Description: PlanningCore ingests SLAM occupancy maps and item targets,
//              builds an internal GridMap, plans routes, and outputs commands.

#ifndef MTRX3760_PLANNING_CORE_H
#define MTRX3760_PLANNING_CORE_H

#include <vector>
#include <optional>
#include <string>

#include "mtrx3760_lrl_package_delivery/path_finder/grid_map.h"
#include "mtrx3760_lrl_package_delivery/path_finder/a_star_planner.h"
#include "mtrx3760_lrl_package_delivery/path_finder/route_solver.h"
#include "mtrx3760_lrl_package_delivery/path_finder/commands.h"

class PlanningCore {
public:
    struct ItemTarget {
        std::string name;
        int x_mm;
        int y_mm;
        char label;
        int priority;
    };

    struct SlamMapProb {
        std::vector<float> prob;   // [0,1] row-major, size rows*cols; negative/NaN => unknown
        int rows;
        int cols;
        double resolution_m;
        float occ_thresh = 0.65f;
        float free_thresh = 0.196f;
        bool unknown_as_occupied = true;
    };

    struct SlamMap100 {
        std::vector<int8_t> occ100; // 0..100, -1 unknown, row-major
        int rows;
        int cols;
        double resolution_m;
        int occ_thresh100 = 65; // >= occupied
        bool unknown_as_occupied = true;
    };

    explicit PlanningCore(double turn_weight_m = 0.0);

    bool linkMap(const SlamMapProb& map);
    bool linkMap(const SlamMap100& map);

    void inflate(double radius_m);

    void setStartCell(int row, int col);
    void setItems(const std::vector<ItemTarget>& items_mm_from_bottom_right);

    bool plan(std::vector<GridMap::Cell>& outFullPath,
              std::vector<GroupReport>& outReports);

    std::vector<MotionCmd> buildCommands(std::optional<int> fixedInitialHeading = std::nullopt) const;

    // Debug helpers
    bool dumpMapCSV(const std::string& path) const { return map_.dumpCSV(path); }
    GridMap::Cell worldMmToCell(int x_mm_from_right, int y_mm_from_bottom) const { return map_.worldMmToCell(x_mm_from_right, y_mm_from_bottom); }

    int rows() const { return map_.rows(); }
    int cols() const { return map_.cols(); }
    double resolution() const { return map_.resolution(); }

private:
    GridMap map_;
    AStarPlanner planner_;
    std::optional<std::vector<GridMap::Cell>> lastFullPath_;
    GridMap::Cell start_{0,0};
    std::vector<Item> items_; // from route_solver.h

    bool loadFromProb_(const SlamMapProb& map);
    bool loadFrom100_(const SlamMap100& map);
};

#endif // MTRX3760_PLANNING_CORE_H


