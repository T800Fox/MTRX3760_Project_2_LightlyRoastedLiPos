// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: grid_map.cpp
// Author: Lightly Roasted Lipos
// Description: Lightweight occupancy grid + world<->grid transforms.

#include "grid_map.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

GridMap::GridMap(double resolution_m_per_cell)
: m_rows(0), m_cols(0), m_res(resolution_m_per_cell), m() {}

bool GridMap::loadCSV(const std::string& path, int rows, int cols)
{
    m_rows = rows;
    m_cols = cols;
    m.assign(rows * cols, 1);

    std::ifstream f(path);
    if (!f.is_open()) return false;

    std::string line;
    int r = 0;
    while (std::getline(f, line) && r < rows) {
        std::istringstream ss(line);
        std::string tok;
        int c = 0;
        while (std::getline(ss, tok, ',') && c < cols) {
            int v = std::stoi(tok);
            m[r * cols + c] = (v <= 0) ? 0 : 1;
            ++c;
        }
        ++r;
    }
    return (r == rows);
}

bool GridMap::isFree(int r, int c) const
{
    if (r < 0 || c < 0 || r >= m_rows || c >= m_cols) return false;
    return m[r * m_cols + c] == 0;
}

GridMap::Cell GridMap::worldMmToCell(int x_mm_from_right, int y_mm_from_bottom) const
{
    const double x_m = x_mm_from_right / 1000.0;
    const double y_m = y_mm_from_bottom / 1000.0;
    const double width_m  = m_cols * m_res;
    const double height_m = m_rows * m_res;
    const double eps = 1e-9;

    const double x_clamped = std::min(std::max(0.0, x_m), width_m  - eps);
    const double y_clamped = std::min(std::max(0.0, y_m), height_m - eps);

    const int col_from_right  = static_cast<int>(std::floor(x_clamped / m_res));
    const int row_from_bottom = static_cast<int>(std::floor(y_clamped / m_res));

    const int col = (m_cols - 1) - col_from_right;
    const int row = (m_rows - 1) - row_from_bottom;
    return { row, col };
}

