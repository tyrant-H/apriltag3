# apriltag3 C++ 重构（使用 OpenCV 数据接口，不直接调用 apriltag3 库）

这个仓库提供了一个 **C/C++ 检测管线实现**，目标是在不直接调用 `apriltag3` 库的前提下，对齐 `apriltag3` 的图像处理和检测识别流程。

## 当前实现

- 输入图像：`cv::Mat`（8-bit 单通道）
- 处理中间结果：`cv::Mat`（梯度图、二值图）
- 输出可视化：`cv::Mat`（检测结果叠加图）
- 预处理：`IntegralImage` + `adaptive_threshold` + `compute_gradient_l1`
- 候选提取：并查集连通域 + quad 候选筛选
- 解码：quad 单应采样 -> border/inner 阈值建模 -> bit 网格 -> 旋转匹配
- tag family：完整 `36h11`（`cv::aruco::DICT_APRILTAG_36h11`）
- 去重：按 `(id + 空间桶)` 聚类，保留最佳候选

## 接口说明

- `AprilTagDetector::detect(const cv::Mat&, PipelineDebugImages*)`
  - 输入：灰度 `cv::Mat`
  - 输出：`std::vector<TagDetection>`
  - debug 输出：`PipelineDebugImages`（`gradient/binary/visualization` 均为 `cv::Mat`）
- `AprilTagDetector::draw_detections(...)` 返回可视化 `cv::Mat`

## 构建与运行

```bash
cmake -S . -B build
cmake --build build
./build/apriltag3_cpp_demo <input_image> [output_vis.png]
```
