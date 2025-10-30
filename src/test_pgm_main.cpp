// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: test_pgm_main.cpp
// Brief: Minimal test driver for PlanningCore using a PGM map input.
// Build:
//   g++ -std=c++17 -O2 \
//       src/test_pgm_main.cpp \
//       src/planning_core.cpp src/grid_map.cpp src/a_star_planner.cpp src/route_solver.cpp src/commands.cpp src/viz.cpp \
//       -I include -o test_pgm
// Usage:
//   ./test_pgm <map.pgm> [resolution_m=0.05] [buffer_m=0.10]

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <cmath>

#include "planning_core.h"

static bool readPGM(const std::string& path, int& outW, int& outH, std::vector<uint8_t>& outData) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::string magic;
    f >> magic;
    if (magic != "P5" && magic != "P2") return false;

    auto skipComments = [&](std::istream& in) {
        int ch = in.peek();
        while (ch == '#') {
            in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            ch = in.peek();
        }
    };

    skipComments(f);
    int w = 0, h = 0, maxv = 255;
    f >> w; skipComments(f);
    f >> h; skipComments(f);
    f >> maxv;
    if (!f.good() || w <= 0 || h <= 0 || maxv <= 0) return false;
    f.get(); // consume one whitespace

    outW = w; outH = h;
    outData.clear(); outData.resize(w * h);

    if (magic == "P5") {
        std::vector<uint8_t> raw(w * h);
        f.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(raw.size()));
        if (!f) return false;
        if (maxv == 255) {
            outData = std::move(raw);
        } else {
            for (int i = 0; i < w*h; ++i) outData[i] = static_cast<uint8_t>(std::lround(255.0 * raw[i] / maxv));
        }
    } else { // P2
        for (int i = 0; i < w * h; ++i) {
            int v = 0; f >> v; if (!f) return false;
            if (maxv != 255) v = static_cast<int>(std::lround(255.0 * v / maxv));
            v = std::clamp(v, 0, 255);
            outData[i] = static_cast<uint8_t>(v);
        }
    }
    return true;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <map.pgm> [resolution_m=0.05] [buffer_m=0.10]\n";
        return 2;
    }
    std::string pgmPath = argv[1];
    double res_m = (argc >= 3) ? std::atof(argv[2]) : 0.05;
    double buffer_m = (argc >= 4) ? std::atof(argv[3]) : 0.10;

    int W = 0, H = 0; std::vector<uint8_t> img;
    if (!readPGM(pgmPath, W, H, img)) {
        std::cerr << "Failed to read PGM: " << pgmPath << "\n";
        return 1;
    }
    std::cout << "Loaded PGM " << W << "x" << H << ", res=" << res_m << " m/cell\n";

    // Map grayscale -> occupancy probability (black=occupied -> p=1, white=free -> p=0)
    std::vector<float> prob(H * W, 1.0f);
    for (int i = 0; i < H * W; ++i) {
        const float g = static_cast<float>(img[i]) / 255.0f;
        prob[i] = 1.0f - g;
    }

    PlanningCore pc(/*turn_weight_m=*/0.10);
    PlanningCore::SlamMapProb slam;
    slam.prob = std::move(prob);
    slam.rows = H;
    slam.cols = W;
    slam.resolution_m = res_m;
    slam.occ_thresh = 0.65f;
    slam.free_thresh = 0.196f;
    slam.unknown_as_occupied = true;

    if (!pc.linkMap(slam)) {
        std::cerr << "linkMap failed\n";
        return 3;
    }

    if (buffer_m > 0.0) pc.inflate(buffer_m);

    // Fixed start to match previous main.cpp example
    pc.setStartCell(/*row=*/10, /*col=*/10);
    std::vector<PlanningCore::ItemTarget> items = {
        {"ItemA", 300, 1800, 'A', 2},
        {"ItemB", 400,  900, 'B', 2},
        {"ItemC", 300,  100, 'C', 1}
    };
    pc.setItems(items);

    std::vector<GridMap::Cell> full;
    std::vector<GroupReport> reps;
    if (!pc.plan(full, reps)) {
        std::cerr << "Planning infeasible\n";
        return 4;
    }

    double total = 0.0; for (const auto& r : reps) total += r.total_m;
    std::cout << "Total traversal: " << total << " m\n";

    auto cmds = pc.buildCommands(/*fixedInitialHeading=*/std::nullopt);
    std::cout << "NAV COMMANDS (Rotate: 0=R 1=L 2=U 3=S, Translate: meters)\n";
    for (const auto& c : cmds) {
        if (c.type == CmdType::Rotate) std::cout << "ROTATE " << c.value << "\n";
        else                           std::cout << "TRANSLATE " << c.value << "\n";
    }

    return 0;
}


