// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: viz.cpp
// Author: Lightly Roasted Lipos
// Description: Simple ASCII visualisation helpers.

#include "viz.h"
#include <iostream>

void printAsciiFull(const GridMap& map,
                    const std::vector<GridMap::Cell>& fullPath,
                    const std::vector<std::pair<char, GridMap::Cell>>& markers)
{
    const int R = map.rows(), C = map.cols();
    std::vector<char> buf(R * C, '.');

    for (int r = 0; r < R; ++r)
        for (int c = 0; c < C; ++c)
            if (!map.isFree(r, c)) buf[r * C + c] = '#';

    for (const auto& p : fullPath)
        buf[p.r * C + p.c] = '*';

    for (const auto& m : markers)
        buf[m.second.r * C + m.second.c] = m.first;

    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) std::cout << buf[r * C + c];
        std::cout << "\n";
    }
}
