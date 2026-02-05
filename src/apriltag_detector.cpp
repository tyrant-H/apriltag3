#include "apriltag3_cpp/apriltag_detector.hpp"

#include "apriltag3_cpp/dedup.hpp"
#include "apriltag3_cpp/decoder.hpp"
#include "apriltag3_cpp/image.hpp"

namespace apriltag3_cpp {

AprilTagDetector::AprilTagDetector(DetectorConfig config) : config_(config) {}

std::vector<QuadCandidate> AprilTagDetector::detect_quads_only(const cv::Mat& image) const {
    const cv::Mat grad = compute_gradient_l1(image);
    const cv::Mat binary = adaptive_threshold(grad, config_.threshold_radius, config_.threshold_offset);
    return detect_quads(binary, config_.min_region_size);
}

std::vector<TagDetection> AprilTagDetector::detect(const cv::Mat& image) const {
    const auto quads = detect_quads_only(image);
    const auto decoded = decode_quads(
        image,
        quads,
        DecoderConfig{config_.decode_tag_data_size, config_.decode_black_border, config_.decode_max_hamming});
    return deduplicate_detections(decoded);
}

}  // namespace apriltag3_cpp
