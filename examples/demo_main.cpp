#include <exception>
#include <iostream>

#include <opencv2/imgcodecs.hpp>

#include "apriltag3_cpp/apriltag_detector.hpp"
#include "apriltag3_cpp/image.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: apriltag3_cpp_demo <input_image> [output_vis.png]\n";
        return 1;
    }

    try {
        const auto image = apriltag3_cpp::load_gray_image(argv[1]);
        apriltag3_cpp::AprilTagDetector detector;
        apriltag3_cpp::PipelineDebugImages debug;

        const auto detections = detector.detect(image, &debug);
        std::cout << "detections_after_decode_dedup=" << detections.size() << "\n";
        for (std::size_t i = 0; i < detections.size(); ++i) {
            const auto& d = detections[i];
            std::cout << "det[" << i << "] id=" << d.id << " hamming=" << d.hamming
                      << " rot=" << d.rotation << " margin=" << d.decision_margin << " area=" << d.area << "\n";
        }

        if (argc >= 3 && !debug.visualization.empty()) {
            if (!cv::imwrite(argv[2], debug.visualization)) {
                std::cerr << "warning: failed to save visualization: " << argv[2] << "\n";
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 2;
    }

    return 0;
}
