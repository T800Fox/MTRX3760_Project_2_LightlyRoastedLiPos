// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: planning_core.cpp
// Description: PlanningCore implementation

#include "X/path_finder/planning_core.h"
#include <cmath>
#include <cstdint>

PlanningCore::PlanningCore(double turn_weight_m)
: map_(0.05), planner_(turn_weight_m) {}

bool PlanningCore::linkMap(const SlamMapProb& sm)
{
    return loadFromProb_(sm);
}

bool PlanningCore::linkMap(const SlamMap100& sm)
{
    return loadFrom100_(sm);
}

void PlanningCore::inflate(double radius_m)
{
    if (radius_m <= 0.0) return;
    const int rad_cells = static_cast<int>(std::ceil(radius_m / map_.resolution()));
    map_.inflateByCells(rad_cells);
}

void PlanningCore::setStartCell(int row, int col)
{
    start_ = {row, col};
}


void PlanningCore::setItems(const std::vector<ItemTarget>& items_mm)
{
    items_.clear();
    items_.reserve(items_mm.size());
    for (const auto& it : items_mm) {
        Item i;
        i.x_mm = it.x_mm;
        i.y_mm = it.y_mm;
        i.priority = it.priority;
        items_.push_back(i);
    }
}

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

std::vector<MotionCmd> PlanningCore::buildCommands(std::optional<int> fixedInitialHeading) const
{
    if (!lastFullPath_.has_value()) return {};
    return buildCommandsFromPath(map_, lastFullPath_.value(), fixedInitialHeading);
}

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


