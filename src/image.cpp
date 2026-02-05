#include "apriltag3_cpp/image.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>

namespace apriltag3_cpp {

namespace {

int clamp_to_byte(int v) {
    if (v < 0) {
        return 0;
    }
    if (v > 255) {
        return 255;
    }
    return v;
}

bool is_gray_u8(const cv::Mat& image) {
    return !image.empty() && image.type() == CV_8UC1;
}

}  // namespace

cv::Mat load_gray_image(const std::string& path) {
    cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (!is_gray_u8(image)) {
        throw std::runtime_error("failed to load 8-bit grayscale image: " + path);
    }
    return image;
}

IntegralImage IntegralImage::build(const cv::Mat& image) {
    IntegralImage ii;
    if (!is_gray_u8(image)) {
        return ii;
    }

    ii.width = image.cols + 1;
    ii.height = image.rows + 1;
    ii.sum.assign(static_cast<std::size_t>(ii.width * ii.height), 0);

    for (int y = 1; y < ii.height; ++y) {
        std::uint32_t row_sum = 0;
        const std::uint8_t* src = image.ptr<std::uint8_t>(y - 1);
        for (int x = 1; x < ii.width; ++x) {
            row_sum += src[x - 1];
            ii.sum[static_cast<std::size_t>(y * ii.width + x)] =
                ii.sum[static_cast<std::size_t>((y - 1) * ii.width + x)] + row_sum;
        }
    }
    return ii;
}

std::uint32_t IntegralImage::rect_sum(int x0, int y0, int x1, int y1) const {
    x0 = std::max(0, std::min(x0, width - 1));
    x1 = std::max(0, std::min(x1, width - 1));
    y0 = std::max(0, std::min(y0, height - 1));
    y1 = std::max(0, std::min(y1, height - 1));

    if (x1 < x0 || y1 < y0) {
        return 0;
    }

    const auto A = sum[static_cast<std::size_t>(y0 * width + x0)];
    const auto B = sum[static_cast<std::size_t>(y0 * width + x1)];
    const auto C = sum[static_cast<std::size_t>(y1 * width + x0)];
    const auto D = sum[static_cast<std::size_t>(y1 * width + x1)];
    return D + A - B - C;
}

cv::Mat compute_gradient_l1(const cv::Mat& image) {
    cv::Mat grad = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    if (!is_gray_u8(image)) {
        return grad;
    }

    for (int y = 1; y < image.rows - 1; ++y) {
        const std::uint8_t* prev = image.ptr<std::uint8_t>(y - 1);
        const std::uint8_t* curr = image.ptr<std::uint8_t>(y);
        const std::uint8_t* next = image.ptr<std::uint8_t>(y + 1);
        std::uint8_t* out = grad.ptr<std::uint8_t>(y);

        for (int x = 1; x < image.cols - 1; ++x) {
            const int gx = static_cast<int>(curr[x + 1]) - static_cast<int>(curr[x - 1]);
            const int gy = static_cast<int>(next[x]) - static_cast<int>(prev[x]);
            const int v = std::abs(gx) + std::abs(gy);
            out[x] = static_cast<std::uint8_t>(clamp_to_byte(v));
        }
    }

    return grad;
}

cv::Mat adaptive_threshold(const cv::Mat& image, int radius, int offset) {
    cv::Mat binary = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    if (!is_gray_u8(image)) {
        return binary;
    }

    const IntegralImage ii = IntegralImage::build(image);

    for (int y = 0; y < image.rows; ++y) {
        const std::uint8_t* src = image.ptr<std::uint8_t>(y);
        std::uint8_t* out = binary.ptr<std::uint8_t>(y);
        for (int x = 0; x < image.cols; ++x) {
            const int x0 = x - radius;
            const int x1 = x + radius + 1;
            const int y0 = y - radius;
            const int y1 = y + radius + 1;
            const int ax0 = std::max(0, x0);
            const int ay0 = std::max(0, y0);
            const int ax1 = std::min(image.cols, x1);
            const int ay1 = std::min(image.rows, y1);
            const int area = (ax1 - ax0) * (ay1 - ay0);

            const std::uint32_t sum = ii.rect_sum(ax0, ay0, ax1, ay1);
            const int mean = (area > 0) ? static_cast<int>(sum / static_cast<std::uint32_t>(area)) : 0;
            out[x] = src[x] > static_cast<std::uint8_t>(mean - offset) ? 255 : 0;
        }
    }

    return binary;
}

}  // namespace apriltag3_cpp
