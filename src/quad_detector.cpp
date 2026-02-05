#include "apriltag3_cpp/quad_detector.hpp"

#include <algorithm>
#include <cstdint>
#include <unordered_map>

#include "apriltag3_cpp/union_find.hpp"

namespace apriltag3_cpp {

namespace {

struct Bounds {
    int min_x;
    int min_y;
    int max_x;
    int max_y;
    int area;
};

bool near_square(const Bounds& b) {
    const int w = b.max_x - b.min_x + 1;
    const int h = b.max_y - b.min_y + 1;
    if (w <= 0 || h <= 0) {
        return false;
    }
    const int larger = std::max(w, h);
    const int smaller = std::min(w, h);
    return smaller * 4 >= larger * 3;
}

bool is_binary_u8(const cv::Mat& image) {
    return !image.empty() && image.type() == CV_8UC1;
}

}  // namespace

std::vector<QuadCandidate> detect_quads(const cv::Mat& binary, int min_region_size) {
    std::vector<QuadCandidate> quads;
    if (!is_binary_u8(binary)) {
        return quads;
    }

    const int width = binary.cols;
    const int height = binary.rows;
    const std::size_t N = static_cast<std::size_t>(width * height);
    UnionFind uf(N);

    auto index = [&](int x, int y) {
        return static_cast<std::size_t>(y * width + x);
    };

    for (int y = 1; y < height; ++y) {
        const std::uint8_t* row = binary.ptr<std::uint8_t>(y);
        const std::uint8_t* prev = binary.ptr<std::uint8_t>(y - 1);
        for (int x = 1; x < width; ++x) {
            if (row[x] == 0) {
                continue;
            }
            if (row[x - 1] != 0) {
                uf.unite(index(x, y), index(x - 1, y));
            }
            if (prev[x] != 0) {
                uf.unite(index(x, y), index(x, y - 1));
            }
        }
    }

    std::unordered_map<std::size_t, Bounds> bounds;
    for (int y = 0; y < height; ++y) {
        const std::uint8_t* row = binary.ptr<std::uint8_t>(y);
        for (int x = 0; x < width; ++x) {
            if (row[x] == 0) {
                continue;
            }
            const auto root = uf.find(index(x, y));
            auto it = bounds.find(root);
            if (it == bounds.end()) {
                bounds.emplace(root, Bounds{x, y, x, y, 1});
            } else {
                Bounds& b = it->second;
                b.min_x = std::min(b.min_x, x);
                b.min_y = std::min(b.min_y, y);
                b.max_x = std::max(b.max_x, x);
                b.max_y = std::max(b.max_y, y);
                b.area += 1;
            }
        }
    }

    quads.reserve(bounds.size());
    for (const auto& [_, b] : bounds) {
        if (b.area < min_region_size || !near_square(b)) {
            continue;
        }
        QuadCandidate q;
        q.corners[0] = {b.min_x, b.min_y};
        q.corners[1] = {b.max_x, b.min_y};
        q.corners[2] = {b.max_x, b.max_y};
        q.corners[3] = {b.min_x, b.max_y};
        q.area = b.area;
        quads.push_back(q);
    }

    return quads;
}

}  // namespace apriltag3_cpp
