// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: route_solver.cpp
// Author: Oliver Lennox
// Description: Computes optimal delivery order and stitched paths across multiple waypoints using a PathPlanner.

#include "mtrx3760_lrl_package_delivery/path_finder/route_solver.h"
#include <algorithm>
#include <limits>

// Constructor links to an existing map and planner implementation (e.g., A* or Theta*)
RouteSolver::RouteSolver(const GridMap& map, const PathPlanner& planner)
: m_map(map), m_planner(planner) {}

// Precompute all pairwise paths ("legs") and their distances between start and item locations
void RouteSolver::buildLegTable(GridMap::Cell start,
                                const std::vector<GridMap::Cell>& itemCells,
                                std::vector<std::vector<LegInfo>>& legs) const
{
    const int N = 1 + static_cast<int>(itemCells.size()); // 0=start
    legs.assign(N, std::vector<LegInfo>(N));

    auto getCell = [&](int idx)->GridMap::Cell {
        return (idx == 0) ? start : itemCells[idx - 1];
    };
    // Plan route between each pair using provided PathPlanner
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            legs[i][j].fromIdx = i;
            legs[i][j].toIdx   = j;
            if (i == j) { legs[i][j].dist_m = 0.0; continue; }
            std::vector<GridMap::Cell> p;
            if (m_planner.plan(m_map, getCell(i), getCell(j), p)) {
                legs[i][j].path   = std::move(p);
                legs[i][j].dist_m = m_planner.pathLength(m_map, legs[i][j].path);
            } else {
                legs[i][j].dist_m = std::numeric_limits<double>::infinity();
            }
        }
    }
}

// Solve the mini travelling-salesman problem for one priority group by testing all permutations
bool RouteSolver::bestTourOver(const std::vector<int>& nodeIds,
                               const std::vector<std::vector<LegInfo>>& legs,
                               std::vector<int>& outOrder,
                               double& outCost) const
{
    // nodeIds contain [1..subset] (item ids). We will compute: 0 -> perm(...) -> 0
    if (nodeIds.empty()) { outOrder = {0,0}; outCost = 0.0; return true; }

    std::vector<int> perm = nodeIds;
    std::sort(perm.begin(), perm.end());
    double best = std::numeric_limits<double>::infinity();
    std::vector<int> bestSeq;

    do {
        double cost = 0.0;
        int prev = 0; // start
        bool ok = true;

        for (int id : perm) {
            const double d = legs[prev][id].dist_m;
            if (!std::isfinite(d)) { ok = false; break; }
            cost += d;
            prev = id;
        }
        if (ok) {
            const double back = legs[prev][0].dist_m;
            if (!std::isfinite(back)) ok = false;
            else cost += back;
        }
        // Keep the best order (lowest total distance) for this group   
        if (ok && cost < best) {
            best = cost;
            bestSeq.clear();
            bestSeq.push_back(0);
            bestSeq.insert(bestSeq.end(), perm.begin(), perm.end());
            bestSeq.push_back(0);
        }
    } while (std::next_permutation(perm.begin(), perm.end()));

    if (!std::isfinite(best)) return false;
    outOrder = bestSeq;
    outCost  = best;
    return true;
}

// High-level routine that groups items by priority, solves each group’s route, and stitches results
bool RouteSolver::solvePriorityGroups(GridMap::Cell start,
                                      const std::vector<Item>& items,
                                      std::vector<GridMap::Cell>& outFullPath,
                                      std::vector<GroupReport>& outReports) const
{
    outFullPath.clear();
    outReports.clear();

    // Convert items to cells
    std::vector<GridMap::Cell> itemCells;
    itemCells.reserve(items.size());
    for (const auto& it : items) {
        itemCells.push_back(m_map.worldMmToCell(it.x_mm, it.y_mm));
    }

    // Precompute all legs
    std::vector<std::vector<LegInfo>> legs;
    // Generate the distance matrix once for all item pairs
    buildLegTable(start, itemCells, legs);

    // Group items by priority (highest first)
    std::map<int, std::vector<int>, std::greater<int>> groups;
    for (int i = 0; i < static_cast<int>(items.size()); ++i)
        groups[items[i].priority].push_back(i + 1); // node id (1..N)

    // Solve each priority group separately (higher priority first)
    for (const auto& [prio, nodeIds] : groups) {
        if (nodeIds.empty()) continue;

        std::vector<int> order; double cost = 0.0;
        if (!bestTourOver(nodeIds, legs, order, cost)) {
            // infeasible group
            return false;
        }

        // Stitch each leg into a continuous full-path for all items
        for (size_t k = 0; k + 1 < order.size(); ++k) {
            const auto& seg = legs[order[k]][order[k+1]].path;
            if (outFullPath.empty()) outFullPath.insert(outFullPath.end(), seg.begin(), seg.end());
            else                     outFullPath.insert(outFullPath.end(), seg.begin()+1, seg.end());
        }

        GroupReport rep;
        rep.priority = prio;
        rep.orderNodeIds = order;
        rep.total_m = cost;
        outReports.push_back(rep);
    }

    return true;
}
