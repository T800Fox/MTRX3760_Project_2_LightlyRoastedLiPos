// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: grid_map.h
// Author: Lightly Roasted Lipos
// Description: Lightweight occupancy grid + world<->grid transforms.

#ifndef MTRX3760_GRID_MAP_H
#define MTRX3760_GRID_MAP_H

#include <string>
#include <vector>

class GridMap {
public:
    struct Cell { int r; int c; };

    explicit GridMap(double resolution_m_per_cell = 0.05);

    // Load CSV with fixed rows/cols
    bool loadCSV(const std::string& path, int rows, int cols);

    // Query
    // Add to GridMap public API
    bool loadCSV(const std::string& path);  // auto-detect rows/cols
    bool isFree(int r, int c) const;
    inline int rows() const { return m_rows; }
    inline int cols() const { return m_cols; }
    inline double resolution() const { return m_res; }

    // Convert world mm (bottom-right origin) → grid cell
    Cell worldMmToCell(int x_mm_from_right, int y_mm_from_bottom) const;

private:
    int m_rows;
    int m_cols;
    double m_res;
    std::vector<int> m; // row-major: 0 free, 1 occupied
};

#endif // MTRX3760_GRID_MAP_H
