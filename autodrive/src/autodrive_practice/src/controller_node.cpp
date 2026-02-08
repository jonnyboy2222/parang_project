#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode() : Node("controller")
  {
    this->declare_parameter<std::string>("topic_stop", "/safety/stop");
    this->declare_parameter<std::string>("topic_speed_cmd", "/cmd/speed");
    this->declare_parameter<double>("default_speed", 5.0);

    topic_stop_ = this->get_parameter("topic_stop").as_string();
    topic_speed_cmd_ = this->get_parameter("topic_speed_cmd").as_string();
    default_speed_ = this->get_parameter("default_speed").as_double();

    pub_speed_ = this->create_publisher<std_msgs::msg::Float32>(topic_speed_cmd_, rclcpp::QoS(10));
    sub_stop_ = this->create_subscription<std_msgs::msg::Bool>(
      topic_stop_, rclcpp::QoS(10),
      std::bind(&ControllerNode::on_stop, this, std::placeholders::_1));

    // 주기적으로 speed_cmd 발행(컨트롤러 느낌)
    timer_ = this->create_wall_timer(100ms, std::bind(&ControllerNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "Sub %s, Pub %s, default_speed=%.2f",
                topic_stop_.c_str(), topic_speed_cmd_.c_str(), default_speed_);
  }

private:
  void on_stop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    stop_ = msg->data;
  }

  void on_timer()
  {
    std_msgs::msg::Float32 cmd;
    cmd.data = stop_ ? 0.0f : static_cast<float>(default_speed_);
    pub_speed_->publish(cmd);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "speed_cmd=%.2f (stop=%s)", cmd.data, stop_ ? "true" : "false");
  }

  std::string topic_stop_;
  std::string topic_speed_cmd_;
  double default_speed_{5.0};
  bool stop_{false};

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
