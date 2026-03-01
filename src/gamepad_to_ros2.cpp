#include <linux/input.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace {
struct AxisRange {
  int min = -24576;
  int max = 24575;
};

double clamp(double value, double lo, double hi) {
  return std::min(std::max(value, lo), hi);
}

double normalize_axis(int value, const AxisRange &range) {
  if (range.max == range.min) {
    return 0.0;
  }
  const double span = static_cast<double>(range.max - range.min);
  const double norm = (2.0 * (static_cast<double>(value) - range.min) / span) - 1.0;
  return clamp(norm, -1.0, 1.0);
}

std::string trim_copy(const std::string &value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string read_device_path_from_config(const std::string &config_path,
                                         const std::string &fallback) {
  std::ifstream file(config_path);
  if (!file.is_open()) {
    return fallback;
  }

  std::string line;
  while (std::getline(file, line)) {
    line = trim_copy(line);
    if (line.empty() || line[0] == '#' || line[0] == ';') {
      continue;
    }

    const auto delimiter = line.find('=');
    if (delimiter == std::string::npos) {
      continue;
    }

    const std::string key = trim_copy(line.substr(0, delimiter));
    const std::string value = trim_copy(line.substr(delimiter + 1));
    if (key == "device_path" && !value.empty()) {
      return value;
    }
  }

  return fallback;
}

bool get_axis_range(int fd, int axis_code, AxisRange *range) {
  if (!range) {
    return false;
  }
  struct input_absinfo abs_info;
  std::memset(&abs_info, 0, sizeof(abs_info));
  if (ioctl(fd, EVIOCGABS(axis_code), &abs_info) == -1) {
    return false;
  }
  range->min = abs_info.minimum;
  range->max = abs_info.maximum;
  return true;
}
}  // namespace

class GamepadToRos2Node : public rclcpp::Node {
public:
  GamepadToRos2Node()
  : rclcpp::Node("gamepad_to_ros2") {
    std::string config_path;
    try {
      const auto share_dir = ament_index_cpp::get_package_share_directory("gamepad4ros2");
      config_path = share_dir + "/config/config.ini";
    } catch (const std::exception &) {
      config_path = "config/config.ini";
    }

    const std::string default_device = read_device_path_from_config(
        config_path, "/dev/input/event16");
    device_path_ = this->declare_parameter<std::string>("device", default_device);
    left_x_code_ = this->declare_parameter<int>("left_x_code", ABS_X);
    right_y_code_ = this->declare_parameter<int>("right_y_code", ABS_RY);
    deadzone_ = this->declare_parameter<double>("deadzone", 0.00);

    steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/control/steering_command", 10);
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/control/throttle_command", 10);
    brake_pub_ = this->create_publisher<std_msgs::msg::Float32>("/control/brake_command", 10);

    open_device();
  }

  ~GamepadToRos2Node() override {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  void spin() {
    while (rclcpp::ok()) {
      if (fd_ < 0) {
        rclcpp::sleep_for(std::chrono::seconds(1));
        open_device();
        continue;
      }

      fd_set read_set;
      FD_ZERO(&read_set);
      FD_SET(fd_, &read_set);

      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 200000;

      const int ready = select(fd_ + 1, &read_set, nullptr, nullptr, &timeout);
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(this->get_logger(), "select() failed: %s", std::strerror(errno));
        close(fd_);
        fd_ = -1;
        continue;
      }

      if (ready == 0) {
        rclcpp::spin_some(this->get_node_base_interface());
        continue;
      }

      if (FD_ISSET(fd_, &read_set)) {
        struct input_event ev;
        const ssize_t bytes = read(fd_, &ev, sizeof(ev));
        if (bytes < 0) {
          if (errno == EAGAIN || errno == EINTR) {
            continue;
          }
          RCLCPP_ERROR(this->get_logger(), "read() failed: %s", std::strerror(errno));
          close(fd_);
          fd_ = -1;
          continue;
        }

        if (bytes != sizeof(ev)) {
          continue;
        }

        if (ev.type == EV_ABS) {
          handle_abs_event(ev.code, ev.value);
        }

        rclcpp::spin_some(this->get_node_base_interface());
      }
    }
  }

private:
  void open_device() {
    fd_ = open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s: %s",
                   device_path_.c_str(), std::strerror(errno));
      return;
    }

    if (!get_axis_range(fd_, left_x_code_, &left_x_range_)) {
      RCLCPP_WARN(this->get_logger(), "Failed to query left_x_code range");
    }
    if (!get_axis_range(fd_, right_y_code_, &right_y_range_)) {
      RCLCPP_WARN(this->get_logger(), "Failed to query right_y_code range");
    }

    publish_throttle_brake(0.0, 0.0);
    publish_steering(0.0);
    RCLCPP_INFO(this->get_logger(), "Reading events from %s", device_path_.c_str());
  }

  void handle_abs_event(int code, int value) {
    if (code == left_x_code_) {
      const double norm = apply_deadzone(normalize_axis(value, left_x_range_));
      //publish_steering(norm);
      const double steering = calculateSteering(norm, 3.0);
      publish_steering(steering);
    } else if (code == right_y_code_) {
      const double norm = apply_deadzone(normalize_axis(value, right_y_range_));
      const double throttle = clamp(-norm, 0.0, 1.0);
      const double brake = clamp(norm, 0.0, 1.0);
      publish_throttle_brake(throttle, brake);
    }
  }

  double apply_deadzone(double value) const {
    if (std::fabs(value) < deadzone_) {
      return 0.0;
    }
    return value;
  }

/* Processes raw controller input into a non-linear steering value.
 * * @param rawInput The raw stick value from [-1.0, 1.0]
 * @param exponent The curve factor (1.0 = linear, 2.0 = quadratic/smooth)
 * @param maxAngle The maximum steering degrees (e.g., 40.0)
 * @return The final steering angle in degrees
 */
double calculateSteering(double rawInput, double exponent) {
    // 1. Clamp input to ensure it's within the [-1.0, 1.0] range
    rawInput = std::clamp(rawInput, -1.0, 1.0);

    // 2. Extract the sign (so we know if we are turning left or right)
    double sign = (rawInput < 0) ? -1.0 : 1.0;

    // 3. Calculate the non-linear curve on the absolute value
    // Formula: y = |x|^n * sign
    double curveValue = std::pow(std::abs(rawInput), exponent) * sign;

    // 4. Map to the car's maximum steering angle
    //return curveValue * maxAngle;
    return curveValue;
  }


  void publish_steering(double value) {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(value);
    steering_pub_->publish(msg);
  }

  void publish_throttle_brake(double throttle, double brake) {
    std_msgs::msg::Float32 throttle_msg;
    std_msgs::msg::Float32 brake_msg;
    throttle_msg.data = static_cast<float>(throttle);
    brake_msg.data = static_cast<float>(brake);
    throttle_pub_->publish(throttle_msg);
    brake_pub_->publish(brake_msg);
  }

  std::string device_path_;
  int left_x_code_ = ABS_X;
  int right_y_code_ = ABS_RY;
  double deadzone_ = 0.00;

  int fd_ = -1;
  AxisRange left_x_range_;
  AxisRange right_y_range_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GamepadToRos2Node>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}
