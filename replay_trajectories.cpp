#include <chrono>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

struct PoseData {
  double x;
  double y;
  double yaw;
};

class CSVCommandPublisher : public rclcpp::Node {
public:
  CSVCommandPublisher(const std::string& csv_file_path)
    : Node("replay_trajectories"), index_(0), frame_rate_(15.0), time_step_(1.0 / 15.0)
  {
    // Load the CSV file
    if (!load_csv(csv_file_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load CSV file.");
      rclcpp::shutdown();
      return;
    }

    // Create publisher for TwistStamped message
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    // Publish at 15 FPS
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(time_step_), std::bind(&CSVCommandPublisher::publish_next_cmd, this));
  }

private:
  bool load_csv(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", file_path.c_str());
      return false;
    }

    std::string line;
    // Skip header
    std::getline(file, line);

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string token;
      PoseData data;
      
      std::getline(ss, token, ',');

      std::getline(ss, token, ',');
      data.x = std::stod(token);
      std::getline(ss, token, ',');
      data.y = std::stod(token);
      std::getline(ss, token, ',');
      data.yaw = std::stod(token);

      data_.push_back(data);
    }

    return !data_.empty();
  }

  void publish_next_cmd() {
    if (index_ >= data_.size() - 1) {
      RCLCPP_INFO(this->get_logger(), "Finished publishing all data.");
      rclcpp::shutdown();
      return;
    }

    auto msg = geometry_msgs::msg::TwistStamped();
    
    // Set timestamp for the message
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";

    // Current pose
    double x1 = data_[index_].x;
    double y1 = data_[index_].y;
    double yaw1 = data_[index_].yaw;

    // Next pose
    double x2 = data_[index_ + 1].x;
    double y2 = data_[index_ + 1].y;
    double yaw2 = data_[index_ + 1].yaw;

    // Calculate distance and angle difference
    double dx = x2 - x1;
    double dy = y2 - y1;
    double distance = std::sqrt(dx * dx + dy * dy);

    double angle_diff = yaw2 - yaw1;

    // Normalize angle_diff to [-pi, pi]
    if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    // Linear velocity: distance / time_step
    msg.twist.linear.x = distance / time_step_;

    // Angular velocity: angle_diff / time_step
    msg.twist.angular.z = angle_diff / time_step_;

    // Publish the message
    publisher_->publish(msg);

    // Move to the next index if we have processed this pose
    index_++;
  }

  std::vector<PoseData> data_;
  size_t index_;
  double frame_rate_;
  double time_step_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: ros2 run <your_package> replay_trajectories <path_to_csv>\n";
    return 1;
  }

  auto node = std::make_shared<CSVCommandPublisher>(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
