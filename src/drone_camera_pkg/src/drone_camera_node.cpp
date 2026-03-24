#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace fs = std::filesystem;

class DroneCameraNode : public rclcpp::Node
{
public:
  DroneCameraNode()
  : Node("drone_camera_node"),
    threshold_cm_(declare_parameter<int>("threshold_cm", 80)),
    camera_device_(declare_parameter<std::string>("camera_device", "/dev/video0")),
    frame_width_(declare_parameter<int>("frame_width", 640)),
    frame_height_(declare_parameter<int>("frame_height", 480)),
    fps_(declare_parameter<double>("fps", 15.0)),
    height_topic_(declare_parameter<std::string>("height_topic", "/height")),
    save_dir_(declare_parameter<std::string>("save_dir", "/home/orangepi/photos")),
    window_name_(declare_parameter<std::string>("window_name", "drone_camera_preview")),
    fine_data_topic_(declare_parameter<std::string>("fine_data_topic", "/fine_data")),
    apriltag_code_topic_(declare_parameter<std::string>("apriltag_code_topic", "/apriltag_code")),
    apriltag_dictionary_name_(
      declare_parameter<std::string>("apriltag_dictionary", "DICT_APRILTAG_36h11")),
    has_height_(false),
    capture_armed_(false),
    pending_capture_(false),
    last_height_cm_(0)
  {
    fs::create_directories(save_dir_);

    height_sub_ = create_subscription<std_msgs::msg::Int16>(
      height_topic_,
      rclcpp::QoS(10),
      std::bind(&DroneCameraNode::heightCallback, this, std::placeholders::_1));

    fine_data_pub_ =
      create_publisher<std_msgs::msg::Int32MultiArray>(fine_data_topic_, rclcpp::QoS(10));
    apriltag_code_pub_ =
      create_publisher<std_msgs::msg::Int32>(apriltag_code_topic_, rclcpp::QoS(10));

    apriltag_dictionary_ = cv::aruco::getPredefinedDictionary(
      dictionaryNameToId(apriltag_dictionary_name_));
    detector_params_ = cv::aruco::DetectorParameters::create();

    if (!camera_.open(camera_device_)) {
      throw std::runtime_error("Failed to open camera device " + camera_device_);
    }

    if (frame_width_ > 0) {
      camera_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    }
    if (frame_height_ > 0) {
      camera_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    }
    if (fps_ > 0.0) {
      camera_.set(cv::CAP_PROP_FPS, fps_);
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(fps_, 1.0));
    frame_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&DroneCameraNode::frameTimerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Camera preview enabled. camera_device=%s threshold_cm=%d save_dir=%s fine_data_topic=%s apriltag_code_topic=%s apriltag_dictionary=%s",
      camera_device_.c_str(),
      threshold_cm_,
      save_dir_.c_str(),
      fine_data_topic_.c_str(),
      apriltag_code_topic_.c_str(),
      apriltag_dictionary_name_.c_str());
  }

  ~DroneCameraNode() override
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (camera_.isOpened()) {
      camera_.release();
    }
    cv::destroyAllWindows();
  }

private:
  static int dictionaryNameToId(const std::string & dictionary_name)
  {
    if (dictionary_name == "DICT_APRILTAG_16h5") {
      return cv::aruco::DICT_APRILTAG_16h5;
    }
    if (dictionary_name == "DICT_APRILTAG_25h9") {
      return cv::aruco::DICT_APRILTAG_25h9;
    }
    if (dictionary_name == "DICT_APRILTAG_36h10") {
      return cv::aruco::DICT_APRILTAG_36h10;
    }
    if (dictionary_name == "DICT_APRILTAG_36h11") {
      return cv::aruco::DICT_APRILTAG_36h11;
    }

    throw std::runtime_error("Unsupported AprilTag dictionary: " + dictionary_name);
  }

  static double contourAreaFromCorners(const std::vector<cv::Point2f> & corners)
  {
    if (corners.size() < 4U) {
      return 0.0;
    }
    return std::abs(cv::contourArea(corners));
  }

  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    const int current_height_cm = static_cast<int>(msg->data);

    if (!has_height_) {
      has_height_ = true;
      last_height_cm_ = current_height_cm;
      if (current_height_cm > threshold_cm_) {
        capture_armed_ = true;
        RCLCPP_INFO(get_logger(), "Capture armed after first height above %d cm.", threshold_cm_);
      }
      return;
    }

    if (current_height_cm > threshold_cm_ && !capture_armed_) {
      capture_armed_ = true;
      RCLCPP_INFO(get_logger(), "Height is above %d cm again. Capture re-armed.", threshold_cm_);
    }

    const bool descending_crossing =
      capture_armed_ &&
      last_height_cm_ > threshold_cm_ &&
      current_height_cm <= threshold_cm_ &&
      current_height_cm < last_height_cm_;

    if (descending_crossing) {
      pending_capture_ = true;
      capture_armed_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Descending across threshold: %d -> %d cm. Capturing one photo.",
        last_height_cm_,
        current_height_cm);
    }

    last_height_cm_ = current_height_cm;
  }

  void detectAprilTagAndPublish(const cv::Mat & frame)
  {
    std::vector<int> tag_ids;
    std::vector<std::vector<cv::Point2f>> tag_corners;

    cv::aruco::detectMarkers(frame, apriltag_dictionary_, tag_corners, tag_ids, detector_params_);
    if (tag_ids.empty()) {
      return;
    }

    std::size_t best_index = 0;
    double best_area = contourAreaFromCorners(tag_corners.front());
    for (std::size_t i = 1; i < tag_corners.size(); ++i) {
      const double area = contourAreaFromCorners(tag_corners[i]);
      if (area > best_area) {
        best_area = area;
        best_index = i;
      }
    }

    const auto & best_corners = tag_corners[best_index];
    const auto center = std::accumulate(
      best_corners.begin(),
      best_corners.end(),
      cv::Point2f(0.0F, 0.0F),
      [](const cv::Point2f & sum, const cv::Point2f & point) {
        return cv::Point2f(sum.x + point.x, sum.y + point.y);
      });

    const float tag_center_x = center.x / static_cast<float>(best_corners.size());
    const float tag_center_y = center.y / static_cast<float>(best_corners.size());
    const float image_center_x = static_cast<float>(frame.cols) / 2.0F;
    const float image_center_y = static_cast<float>(frame.rows) / 2.0F;

    // 用户定义坐标系：上方为 x 正方向，左侧为 y 正方向。
    const int x_offset = static_cast<int>(std::lround(image_center_y - tag_center_y));
    const int y_offset = static_cast<int>(std::lround(image_center_x - tag_center_x));

    std_msgs::msg::Int32MultiArray fine_data_msg;
    fine_data_msg.data = {x_offset, y_offset};
    fine_data_pub_->publish(fine_data_msg);

    std_msgs::msg::Int32 apriltag_code_msg;
    apriltag_code_msg.data = tag_ids[best_index];
    apriltag_code_pub_->publish(apriltag_code_msg);
  }

  void frameTimerCallback()
  {
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!camera_.isOpened()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "Camera is not opened.");
        return;
      }

      if (!camera_.read(frame) || frame.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Failed to read frame from camera.");
        return;
      }
    }

    cv::imshow(window_name_, frame);
    cv::waitKey(1);

    detectAprilTagAndPublish(frame);

    if (pending_capture_) {
      saveFrame(frame);
      pending_capture_ = false;
    }
  }

  void saveFrame(const cv::Mat & frame)
  {
    const auto now = get_clock()->now();
    const auto seconds = now.seconds();
    const auto whole_seconds = static_cast<std::time_t>(seconds);
    const auto millis = static_cast<int>((seconds - static_cast<double>(whole_seconds)) * 1000.0);

    std::tm tm_buf{};
#ifdef _WIN32
    localtime_s(&tm_buf, &whole_seconds);
#else
    localtime_r(&whole_seconds, &tm_buf);
#endif

    std::ostringstream filename;
    filename << "capture_"
             << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
             << '_'
             << std::setw(3) << std::setfill('0') << millis
             << ".jpg";

    const fs::path output_path = fs::path(save_dir_) / filename.str();

    try {
      fs::create_directories(output_path.parent_path());
      if (!cv::imwrite(output_path.string(), frame)) {
        RCLCPP_ERROR(get_logger(), "Failed to save image to %s", output_path.string().c_str());
        return;
      }
      RCLCPP_INFO(get_logger(), "Saved photo: %s", output_path.string().c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Exception while saving image: %s", ex.what());
    }
  }

  int threshold_cm_;
  std::string camera_device_;
  int frame_width_;
  int frame_height_;
  double fps_;
  std::string height_topic_;
  std::string save_dir_;
  std::string window_name_;
  std::string fine_data_topic_;
  std::string apriltag_code_topic_;
  std::string apriltag_dictionary_name_;

  bool has_height_;
  bool capture_armed_;
  bool pending_capture_;
  int last_height_cm_;

  std::mutex frame_mutex_;
  cv::VideoCapture camera_;
  cv::Ptr<cv::aruco::Dictionary> apriltag_dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr apriltag_code_pub_;
  rclcpp::TimerBase::SharedPtr frame_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
