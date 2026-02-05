# apriltag3 C++ 重构（使用 OpenCV 数据接口，不直接调用 apriltag3 库）

这个仓库提供了一个 **C++17 检测管线骨架**，目标是在不直接调用 `apriltag3` 库的前提下，逐步对齐 `apriltag3` 的图像处理与数据处理流程。

## 当前实现

- 图像输入为 `cv::Mat`（8-bit 单通道）
- 积分图与自适应阈值：`IntegralImage` + `adaptive_threshold`
- 梯度计算（L1）：`compute_gradient_l1`
- 连通域（并查集）与四边形候选提取：`detect_quads`
- **decode 阶段（按 apriltag3 流程组织）**
  - quad 建立单应映射（unit square -> quad）
  - 采样 `black_border + data` 网格
  - 用边框/内部灰度模型构建阈值并二值化 data bits
  - 4 方向旋转匹配与最小 Hamming 选择（底层匹配核心改为 C 实现：`decode_core.c`）
- **去重阶段**：`deduplicate_detections`（按 ID+空间桶聚类，保留更优检测）
- 检测入口：`AprilTagDetector::detect`（quad -> decode -> dedup）

> 说明：为保证“流程不简化不改变”，本仓库已将 decode 的低层 bit 旋转/汉明匹配拆到 C 核心模块；当前仍缺少完整 family 字典（如 tag36h11 全码本），该部分将继续按 apriltag3 原流程补齐。

## 约束说明

- ✅ 可以使用 OpenCV / `<cmath>` 等通用库接口。
- ✅ 图像类型统一为 `cv::Mat`。
- ✅ 不直接调用 `apriltag3` 检测 API。

## 构建与运行

```bash
cmake -S . -B build
cmake --build build
./build/apriltag3_cpp_demo <input_image>
```

## 下一步建议（保持速度与准确度）

- 将 codebook 升级为完整 family 字典与真实 bit 位置映射。
- 加入更完整的判决边界模型与解码置信度策略。
- 与 apriltag3 基准集做回归测试（召回率/误检率/FPS），以数据验证不降级。
