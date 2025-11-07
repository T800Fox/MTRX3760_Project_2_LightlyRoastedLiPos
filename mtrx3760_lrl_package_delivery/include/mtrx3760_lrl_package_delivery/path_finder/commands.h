// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: commands.h
// Author: Oliver Lennox
// Description: Build nav-friendly command stream from a stitched cell path.
//        Rotate: value=0 right, 1 left, 2 uturn, 3 straight; Translate: meters.

#ifndef MTRX3760_COMMANDS_H
#define MTRX3760_COMMANDS_H

#include <vector>
#include <optional>
#include "mtrx3760_lrl_package_delivery/path_finder/grid_map.h"

enum class CmdType { Rotate, Translate };

struct MotionCmd {
    CmdType type;
    double value; // Rotate: 0=R,1=L,2=U,3=S ; Translate: meters
};

// Helpers for headings
int headingFromDelta(int dr, int dc);      // 0=N,1=E,2=S,3=W
int turnCode(int h1, int h2);              // 0=R,1=L,2=U,3=S

// Build command list from a 4-connected polyline
std::vector<MotionCmd> buildCommandsFromPath(const GridMap& map,
                                             const std::vector<GridMap::Cell>& fullPath,
                                             std::optional<int> fixedInitialHeading = std::nullopt);

#endif // MTRX3760_COMMANDS_H
