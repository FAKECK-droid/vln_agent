#pragma once
#include <cstdint>
#include <string>
#include <memory>
#include <optional>
#include <vector>
#include <opencv2/core.hpp>
#include "vln_agent/utils/json.hpp"

namespace vln_agent::rgbd
{
/** @brief 返回彩色图网格上的前景或背景提示。 */
struct PromptPoint
{
  int x, y;  ///< 零基像素坐标。
  int label; ///< 1 为前景，0 为背景。
};
/** @brief 单实例提示，框使用图像边界坐标，右/下边界可等于宽/高。 */
struct InstancePrompt
{
  std::vector<PromptPoint> points;  ///< 独立于其他实例的正负点。
  std::optional<cv::Rect2f> box;    ///< 可选 XYXY 框转换出的矩形。
};
/** @brief 原始彩色/对齐深度网格上的单掩码，借用时无需再复制。 */
struct SegmentMask
{
  std::vector<std::uint8_t> pixels; ///< 按行存储的 0/1 掩码。
  cv::Size size;                   ///< 原图尺寸。
  float score{0};                  ///< SAM 预测的掩码质量。
};
/** @brief 串行复用一个 TensorRT 模型和最近一次图像特征。调用由 RgbdRuntime 加锁。 */
class Segmenter
{
public:
  /** @brief 按 warmup_on_initialize 选择初始化预热，或首次 setImage 时载入引擎。 */
  explicit Segmenter(Json settings);
  ~Segmenter();
  /** @brief 原图为 BGR；同一 pair 重用编码，换 pair 时在 CUDA 上重新编码。 */
  void setImage(const std::string & pair_id, const cv::Mat & bgr);
  /** @brief 将返回图片上的提示还原到原图，再单独解码一个实例。 */
  SegmentMask predict(const InstancePrompt & prompt, cv::Size input_size);
private:
  struct Impl;
  Json settings_;                ///< 引擎路径、精度与 CUDA Graph 设置。
  std::unique_ptr<Impl> impl_;   ///< 初始化预热或首次调用时创建，释放时回收 CUDA 资源。
  std::string image_id_;        ///< 仅标识当前驻留特征，不保存历史。
  cv::Size image_size_;         ///< 当前特征对应的原图尺寸。
};
}  // namespace vln_agent::rgbd
