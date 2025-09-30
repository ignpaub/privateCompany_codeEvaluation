#include <chrono>
#include <fstream>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class TfTrajectoryLogger : public rclcpp::Node
{
  public:
    TfTrajectoryLogger(): Node("get_trajectories"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>("parent_frame", "map");
        this->declare_parameter<std::string>("child_frame", "base_link");
        this->declare_parameter<std::string>("output_csv", "trajectory.csv");
        this->declare_parameter<double>("rate", 15.0);  // Hz

        this->get_parameter("parent_frame", parent_frame_);
        this->get_parameter("child_frame", child_frame_);
        this->get_parameter("output_csv", output_csv_);
        double rate_hz;
        this->get_parameter("rate", rate_hz);

        csv_file_.open(output_csv_);
        csv_file_ << "timestamp,x,y,yaw\n";

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz),
            std::bind(&TfTrajectoryLogger::log_transform, this));
    }

    ~TfTrajectoryLogger()
    {
        csv_file_.close();
    }

  private:
    void log_transform()
    {
        try
        {
            auto now = this->get_clock()->now();
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_.lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);

            double x = transform.transform.translation.x;
            double y = transform.transform.translation.y;

            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            csv_file_ << now.seconds() << "," << x << "," << y << "," << yaw << "\n";

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Logged pose: x=%.2f y=%.2f yaw=%.2f", x, y, yaw);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Could not transform %s -> %s: %s", parent_frame_.c_str(), child_frame_.c_str(), ex.what());
        }
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::ofstream csv_file_;

    std::string parent_frame_;
    std::string child_frame_;
    std::string output_csv_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfTrajectoryLogger>());
  rclcpp::shutdown();
  return 0;
}
