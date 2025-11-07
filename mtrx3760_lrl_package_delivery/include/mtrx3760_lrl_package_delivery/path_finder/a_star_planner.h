// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: a_star_planner.h
// Author: Oliver Lennox
// Description: A* implementation (4-connected, Manhattan) with optional turn penalty

#ifndef MTRX3760_A_STAR_PLANNER_H
#define MTRX3760_A_STAR_PLANNER_H

#include "mtrx3760_lrl_package_delivery/path_finder/path_planner.h"

// A* implementation (4-connected, Manhattan) with optional turn penalty
class AStarPlanner final : public PathPlanner {
public:
    explicit AStarPlanner(double turn_weight = 0.0);   // initialise via ctor
    void setTurnWeight(double w) { turn_weight_ = w; } // adjust at runtime

    bool plan(const GridMap& map,
              GridMap::Cell start,
              GridMap::Cell goal,
              std::vector<GridMap::Cell>& outCells) const override;

private:
    double turn_weight_;  // meters of extra cost per heading change
};

#endif // MTRX3760_A_STAR_PLANNER_H
