// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: path_planner.h
// Author: Lightly Roasted Lipos
// Description: Polymorphic path-planning interface.

#ifndef MTRX3760_PATH_PLANNER_H
#define MTRX3760_PATH_PLANNER_H

#include <vector>
#include "grid_map.h"

class PathPlanner {
public:
    virtual ~PathPlanner() = default;

    // Plan a path from start→goal on given map; return true on success.
    // Out path is 4-connected cells including start and goal.
    virtual bool plan(const GridMap& map,
                      GridMap::Cell start,
                      GridMap::Cell goal,
                      std::vector<GridMap::Cell>& outCells) const = 0;

    // Utility: 4-connected path length in meters
    virtual double pathLength(const GridMap& map,
                              const std::vector<GridMap::Cell>& cells) const
    {
        return (cells.size() > 1)
             ? (static_cast<double>(cells.size() - 1) * map.resolution())
             : 0.0;
    }
};

#endif // MTRX3760_PATH_PLANNER_H
