// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: a_star_planner.cpp
// Author: Lightly Roasted Lipos
// Description: A* implementation (4-connected, Manhattan) with optional turn penalty

#include "a_star_planner.h"
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
    if (!map.isFree(start.r,start.c) || !map.isFree(goal.r,goal.c)) return false;

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

    const int sKey = key(start.r,start.c);
    g[sKey] = 0.0;
    open.push({start.r, start.c, -1, h(start.r, start.c)});  // dir=-1 unknown

    const int dr[4] = {-1,1,0,0};
    const int dc[4] = {0,0,-1,1};

    while(!open.empty()){
        Node cur = open.top(); open.pop();
        const int ck = key(cur.r,cur.c);
        if (closed[ck]) continue;
        closed[ck] = 1;

        if (cur.r == goal.r && cur.c == goal.c){
            // reconstruct
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
            // Apply penalty if heading changed
            if (cur.dir != -1 && cur.dir != i) step += (turn_weight_ / map.resolution());
            double ng = g[ck] + step;

            if (ng < g[nk]){
                g[nk] = ng;
                parent[nk] = ck;
                parent_dir[nk] = i;
                open.push({nr, nc, i, ng + h(nr,nc)});
            }
        }
    }
    return false;
}
