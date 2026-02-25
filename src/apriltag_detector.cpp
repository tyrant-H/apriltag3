#include "apriltag3_cpp/apriltag_detector.hpp"

#include <string>

#include <opencv2/imgproc.hpp>

#include "apriltag3_cpp/dedup.hpp"
#include "apriltag3_cpp/decoder.hpp"
#include "apriltag3_cpp/image.hpp"

namespace apriltag3_cpp {

AprilTagDetector::AprilTagDetector(DetectorConfig config) : config_(config) {}

std::vector<QuadCandidate> AprilTagDetector::detect_quads_only(const cv::Mat& image, PipelineDebugImages* debug) const {
    const cv::Mat grad = compute_gradient_l1(image);
    const cv::Mat binary = adaptive_threshold(grad, config_.threshold_radius, config_.threshold_offset);

    if (debug != nullptr) {
        debug->gradient = grad;
        debug->binary = binary;
    }

    return detect_quads(binary, config_.min_region_size);
}

cv::Mat AprilTagDetector::draw_detections(const cv::Mat& image, const std::vector<TagDetection>& detections) const {
    cv::Mat vis;
    if (image.channels() == 1) {
        cv::cvtColor(image, vis, cv::COLOR_GRAY2BGR);
    } else {
        vis = image.clone();
    }

    for (const auto& d : detections) {
        const cv::Scalar color = d.id >= 0 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        for (int i = 0; i < 4; ++i) {
            const auto& a = d.corners[static_cast<std::size_t>(i)];
            const auto& b = d.corners[static_cast<std::size_t>((i + 1) % 4)];
            cv::line(vis, cv::Point(a.x, a.y), cv::Point(b.x, b.y), color, 2, cv::LINE_AA);
        }
        if (d.id >= 0) {
            cv::putText(
                vis,
                "id=" + std::to_string(d.id) + " r=" + std::to_string(d.rotation),
                cv::Point(d.corners[0].x, d.corners[0].y - 4),
                cv::FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv::LINE_AA);
        }
    }
    return vis;
}

std::vector<TagDetection> AprilTagDetector::detect(const cv::Mat& image, PipelineDebugImages* debug) const {
    const auto quads = detect_quads_only(image, debug);
    const auto decoded = decode_quads(
        image,
        quads,
        DecoderConfig{config_.decode_tag_data_size, config_.decode_black_border, config_.decode_max_hamming});
    const auto detections = deduplicate_detections(decoded);

    if (debug != nullptr) {
        debug->visualization = draw_detections(image, detections);
    }
    return detections;
}

}  // namespace apriltag3_cpp
