#include "chrono"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

class TopicsQuiz : public rclcpp::Node {
public:
  TopicsQuiz() : Node("topics_quiz_node") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 1, std::bind(&TopicsQuiz::laser_scanner_reader, this, _1));
    timer_ = this->create_wall_timer(
        50ms, std::bind(&TopicsQuiz::robot_commnader, this));
  }

private:
  void robot_commnader() {
    float left_dis_sensor = 0;
    float right_dis_sensor = 0;
    float mid_dis_sensor = 0;
    if (laser_ranges.size() > 6) {
      for (int i = 0; i <= 19; i++) {
        left_dis_sensor += laser_ranges[200 + i];
        right_dis_sensor += laser_ranges[laser_ranges.size() - i - 200];
        mid_dis_sensor += laser_ranges[std::floor(
            (laser_ranges.size() + std::pow(-1, i) * i) / 2)];
      }
      right_dis_sensor = right_dis_sensor / 20;
      left_dis_sensor = left_dis_sensor / 20;
      mid_dis_sensor = mid_dis_sensor / 20;
    }

    auto command = geometry_msgs::msg::Twist();
    command.linear.y = 0;
    command.linear.z = 0;
    command.angular.x = 0;
    command.angular.y = 0;
    command.linear.x = 0;
    command.angular.z = 0;
    if (right_dis_sensor > 1 && left_dis_sensor > 1 && mid_dis_sensor > 1) {
      command.linear.x = 3;
      command.angular.z = 0;
    } else if (right_dis_sensor < 1) {
      command.linear.x = 0.0;
      command.angular.z = 2;
    } else if (left_dis_sensor < 1) {
      command.linear.x = 0.0;
      command.angular.z = -2;
    } else {
      command.linear.x = 0.0;
      command.angular.z = 2;
    }
    /*for (int i = 0; i <= vector_ranges.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "'%f' ,", vector_ranges[i]);
    }
    RCLCPP_INFO(this->get_logger(), "*********************");*/
    RCLCPP_INFO(this->get_logger(),
                "R : '%f' , M : '%f' , L :'%f' , size: '%ld'", right_dis_sensor,
                mid_dis_sensor, left_dis_sensor, laser_ranges.size());
    publisher_->publish(command);
  }
  void laser_scanner_reader(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laser_ranges = msg->ranges;
  }
  std::vector<float> laser_ranges;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicsQuiz>());
  rclcpp::shutdown();
  return 0;
}