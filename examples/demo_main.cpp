#include <exception>
#include <iostream>

#include "apriltag3_cpp/apriltag_detector.hpp"
#include "apriltag3_cpp/image.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: apriltag3_cpp_demo <input_image>\n";
        return 1;
    }

    try {
        const auto image = apriltag3_cpp::load_gray_image(argv[1]);
        apriltag3_cpp::AprilTagDetector detector;

        const auto quads = detector.detect_quads_only(image);
        std::cout << "quad_candidates=" << quads.size() << "\n";

        const auto detections = detector.detect(image);
        std::cout << "detections_after_decode_dedup=" << detections.size() << "\n";
        for (std::size_t i = 0; i < detections.size(); ++i) {
            const auto& d = detections[i];
            std::cout << "det[" << i << "] id=" << d.id << " hamming=" << d.hamming
                      << " margin=" << d.decision_margin << " area=" << d.area << " corners="
                      << "(" << d.corners[0].x << "," << d.corners[0].y << ") "
                      << "(" << d.corners[1].x << "," << d.corners[1].y << ") "
                      << "(" << d.corners[2].x << "," << d.corners[2].y << ") "
                      << "(" << d.corners[3].x << "," << d.corners[3].y << ")\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 2;
    }

    return 0;
}
