#include "apriltag3_cpp/decoder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

namespace apriltag3_cpp {

namespace {

bool is_gray_u8(const cv::Mat& image) {
    return !image.empty() && image.type() == CV_8UC1;
}

cv::Matx33d homography_unit_to_quad(const QuadCandidate& quad) {
    cv::Mat A(8, 8, CV_64F, cv::Scalar(0));
    cv::Mat b(8, 1, CV_64F, cv::Scalar(0));

    const std::array<cv::Point2d, 4> src = {{
        {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0},
    }};

    for (int i = 0; i < 4; ++i) {
        const double x = src[static_cast<std::size_t>(i)].x;
        const double y = src[static_cast<std::size_t>(i)].y;
        const double u = static_cast<double>(quad.corners[static_cast<std::size_t>(i)].x);
        const double v = static_cast<double>(quad.corners[static_cast<std::size_t>(i)].y);

        A.at<double>(2 * i, 0) = x;
        A.at<double>(2 * i, 1) = y;
        A.at<double>(2 * i, 2) = 1.0;
        A.at<double>(2 * i, 6) = -x * u;
        A.at<double>(2 * i, 7) = -y * u;
        b.at<double>(2 * i, 0) = u;

        A.at<double>(2 * i + 1, 3) = x;
        A.at<double>(2 * i + 1, 4) = y;
        A.at<double>(2 * i + 1, 5) = 1.0;
        A.at<double>(2 * i + 1, 6) = -x * v;
        A.at<double>(2 * i + 1, 7) = -y * v;
        b.at<double>(2 * i + 1, 0) = v;
    }

    cv::Mat h;
    if (!cv::solve(A, b, h, cv::DECOMP_SVD)) {
        return cv::Matx33d::eye();
    }

    return cv::Matx33d(
        h.at<double>(0, 0), h.at<double>(1, 0), h.at<double>(2, 0),
        h.at<double>(3, 0), h.at<double>(4, 0), h.at<double>(5, 0),
        h.at<double>(6, 0), h.at<double>(7, 0), 1.0);
}

int sample_pixel(const cv::Mat& gray, const cv::Matx33d& H, double x, double y) {
    cv::Vec3d p = H * cv::Vec3d(x, y, 1.0);
    if (std::abs(p[2]) < 1e-12) {
        return 0;
    }

    const int u = std::clamp(static_cast<int>(std::lround(p[0] / p[2])), 0, gray.cols - 1);
    const int v = std::clamp(static_cast<int>(std::lround(p[1] / p[2])), 0, gray.rows - 1);
    return static_cast<int>(gray.ptr<std::uint8_t>(v)[u]);
}

TagDetection decode_one(
    const cv::Mat& gray,
    const QuadCandidate& quad,
    const DecoderConfig& config,
    const cv::Ptr<cv::aruco::Dictionary>& family36h11) {
    TagDetection det;
    det.area = quad.area;
    for (int i = 0; i < 4; ++i) {
        det.corners[static_cast<std::size_t>(i)] = quad.corners[i];
    }

    const int d = config.tag_data_size;
    const int border = config.black_border;
    const int total = d + 2 * border;
    if (d <= 0 || total <= 0 || d != 6 || border < 1) {
        return det;
    }

    const cv::Matx33d H = homography_unit_to_quad(quad);

    std::vector<int> sampled(static_cast<std::size_t>(total * total), 0);
    for (int gy = 0; gy < total; ++gy) {
        for (int gx = 0; gx < total; ++gx) {
            const double x = (static_cast<double>(gx) + 0.5) / static_cast<double>(total);
            const double y = (static_cast<double>(gy) + 0.5) / static_cast<double>(total);
            sampled[static_cast<std::size_t>(gy * total + gx)] = sample_pixel(gray, H, x, y);
        }
    }

    int border_sum = 0;
    int border_count = 0;
    int inner_sum = 0;
    int inner_count = 0;
    for (int gy = 0; gy < total; ++gy) {
        for (int gx = 0; gx < total; ++gx) {
            const int v = sampled[static_cast<std::size_t>(gy * total + gx)];
            const bool in_border = (gx < border || gy < border || gx >= (total - border) || gy >= (total - border));
            if (in_border) {
                border_sum += v;
                ++border_count;
            } else {
                inner_sum += v;
                ++inner_count;
            }
        }
    }

    const int border_mean = border_count > 0 ? border_sum / border_count : 0;
    const int inner_mean = inner_count > 0 ? inner_sum / inner_count : border_mean;
    const int threshold = (border_mean + inner_mean) / 2;

    cv::Mat bits(d, d, CV_8UC1, cv::Scalar(0));
    int ones = 0;
    for (int gy = 0; gy < d; ++gy) {
        std::uint8_t* row = bits.ptr<std::uint8_t>(gy);
        for (int gx = 0; gx < d; ++gx) {
            const int v = sampled[static_cast<std::size_t>((gy + border) * total + (gx + border))];
            row[gx] = (v > threshold) ? 1 : 0;
            ones += row[gx];
        }
    }

    int id = -1;
    int rotation = 0;
    const double max_correction_rate = static_cast<double>(config.max_hamming) / static_cast<double>(d * d);
    const bool found = family36h11->identify(bits, id, rotation, max_correction_rate);

    det.id = found ? id : -1;
    det.rotation = found ? rotation : 0;
    det.hamming = found ? family36h11->getDistanceToId(bits, id, true) : (d * d);
    det.decision_margin = static_cast<float>(std::abs((d * d) - 2 * ones));
    return det;
}

}  // namespace

std::vector<TagDetection> decode_quads(
    const cv::Mat& gray,
    const std::vector<QuadCandidate>& quads,
    const DecoderConfig& config) {
    std::vector<TagDetection> detections;
    if (!is_gray_u8(gray)) {
        return detections;
    }

    // Full AprilTag 36h11 tag family from OpenCV predefined dictionary.
    const cv::Ptr<cv::aruco::Dictionary> family36h11 =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

    detections.reserve(quads.size());
    for (const QuadCandidate& quad : quads) {
        detections.push_back(decode_one(gray, quad, config, family36h11));
    }
    return detections;
}

}  // namespace apriltag3_cpp
