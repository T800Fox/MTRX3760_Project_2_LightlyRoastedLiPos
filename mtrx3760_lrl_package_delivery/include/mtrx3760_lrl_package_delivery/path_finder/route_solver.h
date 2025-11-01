// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: route_solver.h
// Author: Lightly Roasted Lipos
// Description: Priority-group mini-TSP solver using a PathPlanner (polymorphic).
//
// For each priority (highest first):
//   Start -> optimal order over that group's items -> return to Start.
// Produces: stitched full path, per-group reports.

#ifndef MTRX3760_ROUTE_SOLVER_H
#define MTRX3760_ROUTE_SOLVER_H

#include <vector>
#include <map>
#include <utility>
#include "grid_map.h"
#include "path_planner.h"

struct Item {
    std::string name;
    int x_mm; int y_mm;
    char label;
    int priority;
};

struct LegInfo {
    int fromIdx; // 0=start, 1..N items
    int toIdx;   // 0=start, 1..N items
    double dist_m;
    std::vector<GridMap::Cell> path;
};

struct GroupReport {
    int priority;
    std::vector<int> orderNodeIds; // e.g., [start(0), i, j, k, 0]
    double total_m{0.0};
};

class RouteSolver {
public:
    RouteSolver(const GridMap& map, const PathPlanner& planner);

    // Compute full traversal with grouped priorities. Returns false if any group infeasible.
    bool solvePriorityGroups(GridMap::Cell start,
                             const std::vector<Item>& items,
                             std::vector<GridMap::Cell>& outFullPath,
                             std::vector<GroupReport>& outReports) const;

private:
    const GridMap& m_map;
    const PathPlanner& m_planner;

    // Precompute all-pairs legs (start+items)
    void buildLegTable(GridMap::Cell start,
                       const std::vector<GridMap::Cell>& itemCells,
                       std::vector<std::vector<LegInfo>>& legs) const;

    // Optimal mini-tour over a subset (start->perm->start), returns nodeId sequence and cost
    bool bestTourOver(const std::vector<int>& nodeIds,
                      const std::vector<std::vector<LegInfo>>& legs,
                      std::vector<int>& outOrder,
                      double& outCost) const;
};

#endif // MTRX3760_ROUTE_SOLVER_H
