#include "apriltag3_cpp/dedup.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace apriltag3_cpp {

namespace {

struct Key {
    int id;
    int cx_bin;
    int cy_bin;

    bool operator==(const Key& other) const {
        return id == other.id && cx_bin == other.cx_bin && cy_bin == other.cy_bin;
    }
};

struct KeyHash {
    std::size_t operator()(const Key& k) const {
        return (static_cast<std::size_t>(k.id) * 73856093u) ^
               (static_cast<std::size_t>(k.cx_bin) * 19349663u) ^
               (static_cast<std::size_t>(k.cy_bin) * 83492791u);
    }
};

Key make_key(const TagDetection& det) {
    float cx = 0.0f;
    float cy = 0.0f;
    for (const auto& c : det.corners) {
        cx += static_cast<float>(c.x);
        cy += static_cast<float>(c.y);
    }
    cx *= 0.25f;
    cy *= 0.25f;

    return Key{
        det.id,
        static_cast<int>(std::floor(cx / 8.0f)),
        static_cast<int>(std::floor(cy / 8.0f)),
    };
}

bool better_detection(const TagDetection& a, const TagDetection& b) {
    if (a.hamming != b.hamming) {
        return a.hamming < b.hamming;
    }
    if (a.decision_margin != b.decision_margin) {
        return a.decision_margin > b.decision_margin;
    }
    return a.area > b.area;
}

}  // namespace

std::vector<TagDetection> deduplicate_detections(std::vector<TagDetection> detections) {
    std::unordered_map<Key, TagDetection, KeyHash> best;
    best.reserve(detections.size());

    for (const auto& det : detections) {
        if (det.id < 0) {
            continue;
        }
        const Key key = make_key(det);
        auto it = best.find(key);
        if (it == best.end() || better_detection(det, it->second)) {
            best[key] = det;
        }
    }

    std::vector<TagDetection> out;
    out.reserve(best.size());
    for (auto& [_, det] : best) {
        out.push_back(det);
    }

    std::sort(out.begin(), out.end(), [](const TagDetection& a, const TagDetection& b) {
        if (a.id != b.id) {
            return a.id < b.id;
        }
        return a.decision_margin > b.decision_margin;
    });
    return out;
}

}  // namespace apriltag3_cpp
