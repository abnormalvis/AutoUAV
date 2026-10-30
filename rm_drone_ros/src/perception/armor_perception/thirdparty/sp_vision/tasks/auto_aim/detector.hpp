#ifndef AUTO_AIM__DETECTOR_HPP
#define AUTO_AIM__DETECTOR_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "armor.hpp"
#include "classifier.hpp"

namespace auto_aim
{

class Detector
{
public:
  Detector(const std::string & config_path, bool debug = true);

  // PATCH(armor_perception): upstream signature was detect(bgr_img, frame_count)
  // and drew the overlay with cv::imshow internally. `debug_img`, when non-null,
  // receives the annotated image instead; nothing is ever displayed.
  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count = -1, cv::Mat * debug_img = nullptr);

  bool detect(Armor & armor, const cv::Mat & bgr_img);

  // PATCH(armor_perception): upstream drew into a window from inside detect().
  // The ROS node needs the annotated image instead, so drawing is a pure
  // function that returns a new image and never touches the GUI.
  cv::Mat draw_result(
    const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars,
    const std::list<Armor> & armors, int frame_count) const;

  friend class YOLOV8;

private:
  Classifier classifier_;

  double threshold_;
  double max_angle_error_;
  double min_lightbar_ratio_, max_lightbar_ratio_;
  double min_lightbar_length_;
  double min_armor_ratio_, max_armor_ratio_;
  double max_side_ratio_;
  double min_confidence_;
  double max_rectangular_error_;

  bool debug_;
  std::string save_path_;
  bool use_classifier_ = true;

  // 利用PCA回归角点，参考自https://github.com/CSU-FYT-Vision/FYT2024_vision
  void lightbar_points_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const;

  bool check_geometry(const Lightbar & lightbar) const;
  bool check_geometry(const Armor & armor) const;
  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  Color get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const;
  cv::Mat get_pattern(const cv::Mat & bgr_img, const Armor & armor) const;
  ArmorType get_type(const Armor & armor);
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  void save(const Armor & armor) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__DETECTOR_HPP