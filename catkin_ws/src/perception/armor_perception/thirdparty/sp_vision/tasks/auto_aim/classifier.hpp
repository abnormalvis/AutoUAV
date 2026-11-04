#ifndef AUTO_AIM__CLASSIFIER_HPP
#define AUTO_AIM__CLASSIFIER_HPP

#include <opencv2/opencv.hpp>
#include <string>

#ifdef ARMOR_USE_OPENVINO
#include <openvino/openvino.hpp>
#endif

#include "armor.hpp"

namespace auto_aim
{
class Classifier
{
public:
  // PATCH(armor_perception): upstream loaded the ONNX model with cv::dnn AND
  // compiled it with OpenVINO in the constructor, so every Detector paid the
  // OpenVINO cost (and needed OpenVINO headers) even when only the traditional
  // light-bar path was used. Both are now lazy: nothing is loaded until
  // classify()/ovclassify() is first called, and the constructor only records
  // the model path. An empty model path disables the classifier, in which case
  // classify() marks the armor `not_armor` instead of crashing.
  explicit Classifier(const std::string & config_path);

  void classify(Armor & armor);

#ifdef ARMOR_USE_OPENVINO
  void ovclassify(Armor & armor);
#endif

private:
  std::string model_path_;

  cv::dnn::Net net_;
  bool net_loaded_ = false;

#ifdef ARMOR_USE_OPENVINO
  ov::Core core_;
  ov::CompiledModel compiled_model_;
  bool ov_loaded_ = false;
#endif
};

}  // namespace auto_aim

#endif  // AUTO_AIM__CLASSIFIER_HPP
