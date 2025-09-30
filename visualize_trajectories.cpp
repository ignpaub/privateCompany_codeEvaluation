#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <fstream>

class TrajectoryFromCSV : public rclcpp::Node
{
public:
  TrajectoryFromCSV()
  : Node("visualize_trajectories")
  {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("robot_trajectory", 10);

    loadCSV("path.csv");  // Change path if needed

    // Publish path repeatedly at 1 Hz (adjust as you want)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TrajectoryFromCSV::publishPath, this));
  }

private:
  void loadCSV(const std::string & filename)
  {
    std::ifstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", filename.c_str());
      return;
    }

    std::string line;
    std::getline(file, line); // skip header

    int idx = 0;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string time_str, x_str, y_str, yaw_str;

      std::getline(ss, time_str, ',');
      std::getline(ss, x_str, ',');
      std::getline(ss, y_str, ',');
      std::getline(ss, yaw_str, ',');

      double x = std::stod(x_str);
      double y = std::stod(y_str);
      double yaw = std::stod(yaw_str);

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = "map";
      pose_stamped.header.stamp = rclcpp::Time(0);

      pose_stamped.pose.position.x = x;
      pose_stamped.pose.position.y = y;
      pose_stamped.pose.position.z = 0.0;

      // Convert yaw to quaternion
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose_stamped.pose.orientation.x = q.x();
      pose_stamped.pose.orientation.y = q.y();
      pose_stamped.pose.orientation.z = q.z();
      pose_stamped.pose.orientation.w = q.w();

      path_.poses.push_back(pose_stamped);
      idx++;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %d poses from CSV.", idx);

    path_.header.frame_id = "map";
  }

  void publishPath()
  {
    path_.header.stamp = this->now();
    path_pub_->publish(path_);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Published path with %zu poses", path_.poses.size());
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryFromCSV>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

