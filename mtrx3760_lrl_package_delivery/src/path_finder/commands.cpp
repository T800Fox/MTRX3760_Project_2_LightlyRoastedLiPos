// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: commands.cpp
// Author: Oliver Lennox
// Description: Build nav-friendly command stream from a stitched cell path.

#include "mtrx3760_lrl_package_delivery/path_finder/commands.h"
#include <cmath>

int headingFromDelta(int dr, int dc) {
    if (dr == -1 && dc == 0) return 0; // N
    if (dr ==  0 && dc == 1) return 1; // E
    if (dr ==  1 && dc == 0) return 2; // S
    if (dr ==  0 && dc == -1) return 3; // W
    return -1;
}

int turnCode(int h1, int h2) {
    int d = (h2 - h1 + 4) % 4;
    switch (d) {
        case 0: return 3; // straight
        case 1: return 1; // left
        case 2: return 2; // u-turn
        case 3: return 0; // right
    }
    return 3;
}

std::vector<MotionCmd> buildCommandsFromPath(const GridMap& map,
                                             const std::vector<GridMap::Cell>& fullPath,
                                             std::optional<int> fixedInitialHeading)
{
    std::vector<MotionCmd> cmds;
    if (fullPath.size() < 2) return cmds;

    int dr0 = fullPath[1].r - fullPath[0].r;
    int dc0 = fullPath[1].c - fullPath[0].c;
    int curHead = headingFromDelta(dr0, dc0);
    if (curHead < 0) return cmds;

    // If robot must start at a fixed heading, rotate first
    if (fixedInitialHeading.has_value()) {
        int t = turnCode(fixedInitialHeading.value(), curHead);
        if (t != 3) cmds.push_back({CmdType::Rotate, static_cast<double>(t)});
    }

    const double cell_m = map.resolution();
    int runSteps = 1;

    for (size_t i = 1; i + 1 < fullPath.size(); ++i) {
        int dr = fullPath[i+1].r - fullPath[i].r;
        int dc = fullPath[i+1].c - fullPath[i].c;
        int nextHead = headingFromDelta(dr, dc);
        if (nextHead < 0) continue;

        if (nextHead == curHead) {
            ++runSteps;
        } else {
            if (runSteps > 0) {
                cmds.push_back({CmdType::Translate, runSteps * cell_m});
                runSteps = 0;
            }
            int t = turnCode(curHead, nextHead);
            cmds.push_back({CmdType::Rotate, static_cast<double>(t)});
            curHead = nextHead;
            runSteps = 1;
        }
    }
    if (runSteps > 0) cmds.push_back({CmdType::Translate, runSteps * cell_m});
    return cmds;
}
