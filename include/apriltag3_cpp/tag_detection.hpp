#pragma once

#include <array>
#include <cstdint>

#include "apriltag3_cpp/quad_detector.hpp"

namespace apriltag3_cpp {

struct TagDetection {
    int id = -1;
    int hamming = 0;
    int rotation = 0;
    float decision_margin = 0.0f;
    int area = 0;
    std::array<Point2i, 4> corners{};
};

}  // namespace apriltag3_cpp
