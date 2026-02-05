#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

namespace apriltag3_cpp {

struct Point2i {
    int x = 0;
    int y = 0;
};

struct QuadCandidate {
    Point2i corners[4];
    int area = 0;
};

std::vector<QuadCandidate> detect_quads(const cv::Mat& binary, int min_region_size);

}  // namespace apriltag3_cpp
