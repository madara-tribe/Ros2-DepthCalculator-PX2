#include "sw_px2/inference.h"
#include "sw_px2/tool.h"

#define ONNX_DEPTH_PATH "/weights/dpt_large_384.onnx"
#define ONNX_YOLO_PATH "/weights/yolov7Tiny_640_640.onnx"
#define CSV_PATH "data/results.csv"
#define YOLO_INPUT_H 640
#define YOLO_INPUT_W 640

using namespace std::chrono_literals;

namespace onnx_inference
{
  OnnxInferenceNode::OnnxInferenceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("px2", options)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("inference", rclcpp::QoS{3}.transient_local()); 
    timer_ = this->create_wall_timer(
        500ms, std::bind(&OnnxInferenceNode::callbackInference, this));
  }
  
  void OnnxInferenceNode::callbackInference()
  {
      bool useCUDA{false};
      std::cout << "Midas Prepare" << std::endl;
      MidasInference midas(pkg_path + ONNX_DEPTH_PATH, useCUDA); // Midas instance
      std::cout << "YOLO Prepare" << std::endl;
      YoloDetect yolo_detector(pkg_path + ONNX_YOLO_PATH); // YOLO instance
      	  
      deleteCSV(pkg_path + CSV_PATH);

      std::vector<std::string> img_filenames = ListImages(pkg_path + "data");
      if (img_filenames.size() !=0){
        for (const auto& file : img_filenames) {
          std::cout << "Found image: " << file << std::endl;
        }
	image_num = img_filenames.size();
	std::cout << "total image : " << img_filenames.size() << std::endl;
      } else {
	  std::cout << "\nError: Not images exist in " << pkg_path + "data" << std::endl;
      }
      auto start = std::chrono::high_resolution_clock::now();
      for(int i=0; i<image_num; i++){
        cv::Mat yolo_image = cv::imread(img_filenames[i]);
        cv::Mat midas_image = cv::imread(img_filenames[i]);
        std::cout << "img size " << yolo_image.channels() << "w" <<yolo_image.cols<< "h" << yolo_image.rows << std::endl; 
	// yolo
        cv::Mat inputImage = yolo_detector.preprocess(yolo_image, YOLO_INPUT_H, YOLO_INPUT_W);
        std::vector<Ort::Value> outputTensors = yolo_detector.RunInference(inputImage);
        std::vector<Result> resultVector = yolo_detector.postprocess(yolo_image.size(), outputTensors);
        cv::Mat yolo_result = yolo_detector.drawBoundingBox(yolo_image, resultVector);
	// midas
        cv::Mat depth_resize = midas.runInference(midas_image, pkg_path);
        
        image_h = depth_resize.rows;
        image_w = depth_resize.cols;	
        SearchMedian(img_filenames[i], resultVector, depth_resize, yolo_result);
      }
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> diff = end - start;
      std::cout << "Prediction took " << diff.count() << " seconds" << std::endl;
      std::string csvFilePath = pkg_path + CSV_PATH;
      saveToCSV(csvFilePath, csv_data);
      std::cout << "CSV data saved successfully!" << std::endl;
      
      std_msgs::msg::String message;
      message.data = std::to_string(diff.count());
      publishState(message);		 

      rclcpp::shutdown(); // Shut down the node after publishing
  }
  
  void OnnxInferenceNode::SearchMedian(std::string imgPath, std::vector<Result> resultVector, cv::Mat& depth_resize, cv::Mat& yolo_result){
    size_t lastSlash = imgPath.find_last_of("/");
    size_t lastDot = imgPath.find_last_of(".");
    std::string baseName = imgPath.substr(lastSlash + 1, lastDot - lastSlash - 1);
    std::string outputPath = "data/" + baseName + "_result.png";
    for( auto result : resultVector ) {
    double x = result.x1;
    double y = result.y1;
    double w = result.x2;
    double h = result.y2;
    try {
      cv::Mat cropDepth = depth_resize(cv::Range(y, h), cv::Range(x, w)).clone();
      float medianValue = computeMedian(cropDepth);
      float bbox_size = calculateBboxSize(x, y, w, h);
      double x_real_coordinate = calculateRealCoordinate(x, w, image_w);
      std::cout << "Horizontal real distance: " << x_real_coordinate << "cm" << std::endl;
      std::cout << "median" << medianValue << "bbox size " << bbox_size << std::endl;
      cv::putText(yolo_result, std::to_string(bbox_size),
        cv::Point(x, y+50), cv::FONT_ITALIC,
        0.8, cv::Scalar(255, 255, 0), 2);
      csv_data.push_back({outputPath, std::to_string(image_h), std::to_string(image_w), std::to_string((w+x)/2),
                                std::to_string((y+h)/2),
                                std::to_string(medianValue), std::to_string(bbox_size), std::to_string(x_real_coordinate)});
    } catch (const cv::Exception& e){
      // std::cerr << "opencv error"<< e.what() << std::endl;
    }
    }
    cv::imwrite(pkg_path + outputPath, yolo_result);
    std::cout << "succeeded to save as "<< pkg_path + outputPath << std::endl;
  }
  void OnnxInferenceNode::publishState(std_msgs::msg::String message_)
  {
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_.data.c_str());
    publisher_->publish(message_);
  }
  
  float OnnxInferenceNode::computeMedian(cv::Mat& img) {
    // Flatten image pixels into vector
    std::vector<uchar> pixels(img.datastart, img.dataend);
    size_t n = pixels.size();
    if (n == 0) return -1.0f;

    auto mid_iter = pixels.begin() + n / 2;
    std::nth_element(pixels.begin(), mid_iter, pixels.end());
    float median = static_cast<float>(*mid_iter);

    // If even, also find the largest element in the lower half to average with median
    if (n % 2 == 0) {
        auto lower_max = std::max_element(pixels.begin(), mid_iter);
        median = (median + static_cast<float>(*lower_max)) / 2.0f;
    }
    return median;
  }  
}  // namespace onnx_inference


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<onnx_inference::OnnxInferenceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

