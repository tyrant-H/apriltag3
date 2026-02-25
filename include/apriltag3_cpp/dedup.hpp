#pragma once

#include <vector>

#include "apriltag3_cpp/tag_detection.hpp"

namespace apriltag3_cpp {

std::vector<TagDetection> deduplicate_detections(std::vector<TagDetection> detections);

}  // namespace apriltag3_cpp
