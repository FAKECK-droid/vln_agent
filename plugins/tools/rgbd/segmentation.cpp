#include "vln_agent/plugins/tools/rgbd/segmentation.hpp"
#include <algorithm>
#include <stdexcept>
#include <opencv2/imgproc.hpp>
#ifdef VLN_AGENT_HAS_MOBILE_SAM
#include <mobile_sam_trt/mobile_sam.hpp>
#endif

namespace vln_agent::rgbd
{
#ifdef VLN_AGENT_HAS_MOBILE_SAM
namespace
{
mobile_sam_trt::Precision precision(const std::string & name)
{
  if (name == "fp16") {return mobile_sam_trt::Precision::FP16;}
  if (name == "tf32") {return mobile_sam_trt::Precision::TF32;}
  if (name == "fp32") {return mobile_sam_trt::Precision::FP32;}
  throw std::invalid_argument("SAM 精度使用 fp16、tf32 或 fp32");
}
}
struct Segmenter::Impl
{
  explicit Impl(const Json & settings)
  : model(settings.at("engine_directory").get<std::string>(),
      precision(settings.value("encoder_precision", std::string("tf32"))),
      precision(settings.value("decoder_precision", std::string("fp32"))),
      settings.value("cuda_graph", true)) {}
  mobile_sam_trt::MobileSam model; ///< 引擎、流、缓冲区及图像特征由库复用。
};
#else
struct Segmenter::Impl {};
#endif

Segmenter::Segmenter(Json settings) : settings_(std::move(settings))
{
#ifdef VLN_AGENT_HAS_MOBILE_SAM
  if (settings_.value("warmup_on_initialize", false)) {
    // 只预热引擎和常用提示形状，临时图像不进入 RGB-D pair 缓存。
    const cv::Size size(1024, 1024);
    setImage("__warmup__", cv::Mat(size, CV_8UC3, cv::Scalar(0, 0, 0)));
    InstancePrompt prompt;
    prompt.points.push_back({512, 512, 1});
    predict(prompt, size);  // 正点 + padding：2 个 token。
    prompt.box = cv::Rect2f(256, 256, 512, 512);
    predict(prompt, size);  // 正点 + 框：3 个 token。
    image_id_.clear();     // 第一张真实图片必须重新编码。
  }
#endif
}
Segmenter::~Segmenter() = default;

void Segmenter::setImage(const std::string & pair_id, const cv::Mat & bgr)
{
#ifdef VLN_AGENT_HAS_MOBILE_SAM
  if (!impl_) {impl_ = std::make_unique<Impl>(settings_);}
  if (image_id_ == pair_id) {return;}
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  impl_->model.setImage(rgb.data, rgb.cols, rgb.rows);
  image_size_ = rgb.size();
  image_id_ = pair_id;
#else
  (void)pair_id; (void)bgr;
  throw std::runtime_error("当前构建未链接 mobile_sam_trt");
#endif
}

SegmentMask Segmenter::predict(const InstancePrompt & input, cv::Size input_size)
{
#ifdef VLN_AGENT_HAS_MOBILE_SAM
  const float sx = float(image_size_.width) / input_size.width;
  const float sy = float(image_size_.height) / input_size.height;
  mobile_sam_trt::Prompts prompt;
  prompt.points.reserve(input.points.size());
  for (const auto & p : input.points) {
    prompt.points.push_back({std::clamp((p.x + 0.5f) * sx - 0.5f, 0.f,
        float(image_size_.width - 1)),
      std::clamp((p.y + 0.5f) * sy - 0.5f, 0.f, float(image_size_.height - 1)), p.label});
  }
  if (input.box) {
    const auto & b = *input.box;
    prompt.has_box = true;
    prompt.box = {b.x * sx, b.y * sy, (b.x + b.width) * sx, (b.y + b.height) * sy};
  }
  auto result = impl_->model.predict(prompt, false, false);
  return {std::move(result.masks), {result.width, result.height}, result.scores.at(0)};
#else
  (void)input; (void)input_size;
  throw std::runtime_error("当前构建未链接 mobile_sam_trt");
#endif
}
}  // namespace vln_agent::rgbd
