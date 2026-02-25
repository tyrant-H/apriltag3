#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace apriltag3_cpp {

cv::Mat load_gray_image(const std::string& path);

struct IntegralImage {
    int width = 0;
    int height = 0;
    std::vector<std::uint32_t> sum;

    static IntegralImage build(const cv::Mat& image);
    std::uint32_t rect_sum(int x0, int y0, int x1, int y1) const;
};

cv::Mat compute_gradient_l1(const cv::Mat& image);
cv::Mat adaptive_threshold(const cv::Mat& image, int radius, int offset);

}  // namespace apriltag3_cpp
