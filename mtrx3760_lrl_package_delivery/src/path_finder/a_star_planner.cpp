// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: a_star_planner.cpp
// Author: Lightly Roasted Lipos
// Description: A* implementation (4-connected, Manhattan) with optional turn penalty

#include "mtrx3760_lrl_package_delivery/path_finder/a_star_planner.h"
#include <queue>
#include <limits>
#include <cmath>

AStarPlanner::AStarPlanner(double turn_weight)
: turn_weight_(turn_weight) {}

bool AStarPlanner::plan(const GridMap& map,
                        GridMap::Cell start,
                        GridMap::Cell goal,
                        std::vector<GridMap::Cell>& outCells) const
{
    outCells.clear();

    const int R = map.rows(), C = map.cols();
    if (!map.isFree(start.r,start.c)) {
        // Snap start to nearest free cell if inflated map covers original start
        int bestR = -1, bestC = -1; double bestD2 = std::numeric_limits<double>::infinity();
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                if (!map.isFree(r,c)) continue;
                double d2 = static_cast<double>(r - start.r) * (r - start.r) + static_cast<double>(c - start.c) * (c - start.c);
                if (d2 < bestD2) { bestD2 = d2; bestR = r; bestC = c; }
            }
        }
        if (bestR == -1) return false;
        start = {bestR, bestC};
    }

    auto key = [&](int r,int c){ return r*C + c; };

    struct Node {
        int r,c,dir;
        double f;
        bool operator<(const Node& o) const { return f > o.f; }
    };

    std::priority_queue<Node> open;
    std::vector<char> closed(R*C, 0);
    std::vector<double> g(R*C, std::numeric_limits<double>::infinity());
    std::vector<int> parent(R*C, -1);
    std::vector<int> parent_dir(R*C, -1);

    auto h = [&](int r,int c){
        return static_cast<double>(std::abs(r - goal.r) + std::abs(c - goal.c));
    };

    auto runAStar = [&](GridMap::Cell useGoal)->bool {
        std::fill(closed.begin(), closed.end(), 0);
        std::fill(g.begin(), g.end(), std::numeric_limits<double>::infinity());
        std::fill(parent.begin(), parent.end(), -1);
        std::fill(parent_dir.begin(), parent_dir.end(), -1);
        while (!open.empty()) open.pop();

        auto h_local = [&](int r,int c){
            return static_cast<double>(
                std::abs(r - useGoal.r) + std::abs(c - useGoal.c)
            );
        };

        const int sKey = key(start.r,start.c);
        g[sKey] = 0.0;
        open.push({start.r, start.c, -1, h_local(start.r, start.c)});  // dir=-1 unknown

        const int dr[4] = {-1,1,0,0};
        const int dc[4] = {0,0,-1,1};

        while(!open.empty()){
            Node cur = open.top(); open.pop();
            const int ck = key(cur.r,cur.c);
            if (closed[ck]) continue;
            closed[ck] = 1;

            if (cur.r == useGoal.r && cur.c == useGoal.c){
                std::vector<GridMap::Cell> rev;
                for (int k = ck; k != -1; k = parent[k]) {
                    rev.push_back({k / C, k % C});
                }
                outCells.assign(rev.rbegin(), rev.rend());
                return true;
            }

            for (int i = 0; i < 4; ++i){
                const int nr = cur.r + dr[i];
                const int nc = cur.c + dc[i];
                if (nr < 0 || nc < 0 || nr >= R || nc >= C) continue;
                if (!map.isFree(nr,nc)) continue;
                const int nk = key(nr,nc);
                if (closed[nk]) continue;

                double step = 1.0; // base grid step
                if (cur.dir != -1 && cur.dir != i) step += (turn_weight_ / map.resolution());
                double ng = g[ck] + step;

                if (ng < g[nk]){
                    g[nk] = ng;
                    parent[nk] = ck;
                    parent_dir[nk] = i;
                    open.push({nr, nc, i, ng + h_local(nr,nc)});
                }
            }
        }
        return false;
    };

    // If goal is free, try normal A*
    if (map.isFree(goal.r, goal.c)) {
        if (runAStar(goal)) return true;
    }

    // Fallback: goal blocked or unreachable -> route to closest reachable free cell to goal
    // BFS from start over free cells to find reachable set
    std::vector<char> seen(R*C, 0);
    std::vector<int> q; q.reserve(R*C);
    auto push = [&](int r,int c){ int k = key(r,c); if (!seen[k]) { seen[k] = 1; q.push_back(k); } };
    push(start.r, start.c);
    for (size_t qi = 0; qi < q.size(); ++qi) {
        int k = q[qi]; int r = k / C, c = k % C;
        const int dr4[4] = {-1,1,0,0};
        const int dc4[4] = {0,0,-1,1};
        for (int i = 0; i < 4; ++i) {
            int nr = r + dr4[i], nc = c + dc4[i];
            if (nr < 0 || nc < 0 || nr >= R || nc >= C) continue;
            if (!map.isFree(nr,nc)) continue;
            push(nr,nc);
        }
    }

    // Choose reachable cell with minimal Euclidean distance to intended goal
    int bestK = -1; double bestD2 = std::numeric_limits<double>::infinity();
    for (int k = 0; k < R*C; ++k) {
        if (!seen[k]) continue;
        int r = k / C, c = k % C;
        double d2 = static_cast<double>(r - goal.r) * (r - goal.r) + static_cast<double>(c - goal.c) * (c - goal.c);
        if (d2 < bestD2) { bestD2 = d2; bestK = k; }
    }
    if (bestK == -1) return false; // nowhere reachable

    GridMap::Cell nearGoal{bestK / C, bestK % C};
    return runAStar(nearGoal);
}
