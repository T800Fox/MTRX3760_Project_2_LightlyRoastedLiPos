// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: viz.h
// Author: Lightly Roasted Lipos
// Description: Simple ASCII visualisation helpers.

#ifndef MTRX3760_VIZ_H
#define MTRX3760_VIZ_H

#include <vector>
#include <utility>
#include "grid_map.h"

void printAsciiFull(const GridMap& map,
                    const std::vector<GridMap::Cell>& fullPath,
                    const std::vector<std::pair<char, GridMap::Cell>>& markers);

#endif // MTRX3760_VIZ_H
