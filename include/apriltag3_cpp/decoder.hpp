#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "apriltag3_cpp/quad_detector.hpp"
#include "apriltag3_cpp/tag_detection.hpp"

namespace apriltag3_cpp {

struct DecoderConfig {
    int tag_data_size = 6;
    int black_border = 1;
    int max_hamming = 5;
};

std::vector<TagDetection> decode_quads(
    const cv::Mat& gray,
    const std::vector<QuadCandidate>& quads,
    const DecoderConfig& config = {});

}  // namespace apriltag3_cpp
