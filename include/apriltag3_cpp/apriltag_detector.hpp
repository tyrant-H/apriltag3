#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "apriltag3_cpp/quad_detector.hpp"
#include "apriltag3_cpp/tag_detection.hpp"

namespace apriltag3_cpp {

struct DetectorConfig {
    int threshold_radius = 8;
    int threshold_offset = 7;
    int min_region_size = 24;
    int decode_tag_data_size = 6;
    int decode_black_border = 1;
    int decode_max_hamming = 5;
};

class AprilTagDetector {
public:
    explicit AprilTagDetector(DetectorConfig config = {});
    std::vector<QuadCandidate> detect_quads_only(const cv::Mat& image) const;
    std::vector<TagDetection> detect(const cv::Mat& image) const;

private:
    DetectorConfig config_;
};

}  // namespace apriltag3_cpp
