// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: planning_core.cpp
// Author: Oliver Lennox
// Description: Integrates map data, route solving, and command generation into a single planning interface.

#include "mtrx3760_lrl_package_delivery/path_finder/planning_core.h"
#include <cmath>
#include <cstdint>

// Initialise map with default resolution and planner with turn weight penalty
PlanningCore::PlanningCore(double turn_weight_m)
: map_(0.05), planner_(turn_weight_m) {}

// Link a probability-based SLAM map (0.0–1.0) to GridMap representation
bool PlanningCore::linkMap(const SlamMapProb& sm)
{
    return loadFromProb_(sm);
}

// Link a 0–100 occupancy grid map (ROS format) to GridMap
bool PlanningCore::linkMap(const SlamMap100& sm)
{
    return loadFrom100_(sm);
}

// Expand occupied cells by a safety radius (in meters) to prevent collisions
void PlanningCore::inflate(double radius_m)
{
    if (radius_m <= 0.0) return;
    const int rad_cells = static_cast<int>(std::ceil(radius_m / map_.resolution()));
    map_.inflateByCells(rad_cells);
}

// Define robot starting location in grid coordinates
void PlanningCore::setStartCell(int row, int col)
{
    start_ = {row, col};
}

// Convert item positions from world millimetres into internal item structures
void PlanningCore::setItems(const std::vector<ItemTarget>& items_mm)
{
    items_.clear();
    items_.reserve(items_mm.size());
    for (const auto& it : items_mm) {
        Item i; i.name = it.name; i.x_mm = it.x_mm; i.y_mm = it.y_mm; i.label = it.label; i.priority = it.priority;
        items_.push_back(i);
    }
}

// Run the RouteSolver using the chosen PathPlanner and return the full delivery route
bool PlanningCore::plan(std::vector<GridMap::Cell>& outFullPath,
                        std::vector<GroupReport>& outReports)
{
    RouteSolver solver(map_, planner_);
    std::vector<GridMap::Cell> full;
    std::vector<GroupReport> reps;
    if (!solver.solvePriorityGroups(start_, items_, full, reps)) return false;
    lastFullPath_ = full;
    outFullPath.swap(full);
    outReports.swap(reps);
    return true;
}

// Convert the computed path into Rotate/Translate motion commands for actuators
std::vector<MotionCmd> PlanningCore::buildCommands(std::optional<int> fixedInitialHeading) const
{
    if (!lastFullPath_.has_value()) return {};
    return buildCommandsFromPath(map_, lastFullPath_.value(), fixedInitialHeading);
}

// Internal: translate probability SLAM map to binary occupancy map
bool PlanningCore::loadFromProb_(const SlamMapProb& sm)
{
    if (sm.rows <= 0 || sm.cols <= 0) return false;
    if (static_cast<int>(sm.prob.size()) != sm.rows * sm.cols) return false;
    std::vector<int> bin(sm.rows * sm.cols, 1);
    for (int i = 0; i < sm.rows * sm.cols; ++i) {
        const float p = sm.prob[i];
        const bool unknown = !(p >= 0.0f && p <= 1.0f);
        if (unknown) {
            bin[i] = sm.unknown_as_occupied ? 1 : 0;
        } else if (p > sm.occ_thresh) {
            bin[i] = 1;
        } else if (p < sm.free_thresh) {
            bin[i] = 0;
        } else {
            bin[i] = 1; // treat gray as occupied by default
        }
    }
    return map_.loadFromBinary(bin, sm.rows, sm.cols, sm.resolution_m);
}

// Internal: translate 0–100 occupancy grid map to binary occupancy map
bool PlanningCore::loadFrom100_(const SlamMap100& sm)
{
    if (sm.rows <= 0 || sm.cols <= 0) return false;
    if (static_cast<int>(sm.occ100.size()) != sm.rows * sm.cols) return false;
    std::vector<int> bin(sm.rows * sm.cols, 1);
    for (int i = 0; i < sm.rows * sm.cols; ++i) {
        const int v = static_cast<int>(sm.occ100[i]);
        if (v < 0) { // unknown
            bin[i] = sm.unknown_as_occupied ? 1 : 0;
        } else if (v >= sm.occ_thresh100) {
            bin[i] = 1;
        } else {
            bin[i] = 0;
        }
    }
    return map_.loadFromBinary(bin, sm.rows, sm.cols, sm.resolution_m);
}


