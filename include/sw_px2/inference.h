#ifndef ONNX_INFERENCE_HPP_
#define ONNX_INFERENCE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <cstdio>  // For std::remove()

#include "yolo_inference.h"
#include "midas_inference.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#define PKG_PATH "/ros2_ws/space/src/sw_px2/"

namespace onnx_inference
{
class OnnxInferenceNode : public rclcpp::Node
{
public:
  explicit OnnxInferenceNode(const rclcpp::NodeOptions & options);
private:
  // Subscriber
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  
  std::string pkg_path = PKG_PATH; //ament_index_cpp::get_package_share_directory("leastsquares");
  int image_num;
  int image_h;
  int image_w;
  std::vector<std::vector<std::string>> csv_data;
  void callbackInference();
  float computeMedian(cv::Mat& img);
  void SearchMedian(std::string imgPath, std::vector<Result> resultVector, cv::Mat& depth_resize, cv::Mat& yolo_result);
  void publishState(std_msgs::msg::String message);
};

}  // namespace onnx_inference
#endif  // ONN_INFEREMNCE_HPP_


