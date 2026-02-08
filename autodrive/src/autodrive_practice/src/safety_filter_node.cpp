#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include "autodrive_practice/safety_filter_core.hpp"

class SafetyFilterNode : public rclcpp::Node
{
public:
  SafetyFilterNode() : Node("safety_filter")
  {
    // parameters
    this->declare_parameter<std::string>("topic_in", "/fake/lidar_distance_m");
    this->declare_parameter<std::string>("topic_stop", "/safety/stop");
    this->declare_parameter<double>("stop_threshold_m", 2.0);

    topic_in_ = this->get_parameter("topic_in").as_string();
    topic_stop_ = this->get_parameter("topic_stop").as_string();

    core_.set_stop_threshold(static_cast<float>(this->get_parameter("stop_threshold_m").as_double()));

    // publishers/subscribers
    pub_stop_ = this->create_publisher<std_msgs::msg::Bool>(topic_stop_, rclcpp::QoS(10));

    sub_ = this->create_subscription<std_msgs::msg::Float32>(
      topic_in_, rclcpp::QoS(10),
      std::bind(&SafetyFilterNode::on_distance, this, std::placeholders::_1));

    // parameter callback (런타임에서 threshold 바꿀 수 있게)
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SafetyFilterNode::on_params, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing %s, publishing %s, threshold=%.2f m",
                topic_in_.c_str(), topic_stop_.c_str(), core_.stop_threshold());
  }

private:
  rcl_interfaces::msg::SetParametersResult on_params(const std::vector<rclcpp::Parameter>& params)
  {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    for (const auto& p : params) {
      if (p.get_name() == "stop_threshold_m") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          res.successful = false;
          res.reason = "stop_threshold_m must be double";
          return res;
        }
        core_.set_stop_threshold(static_cast<float>(p.as_double()));
        RCLCPP_INFO(get_logger(), "Updated stop_threshold_m=%.2f", core_.stop_threshold());
      }
    }
    return res;
  }

  void on_distance(const std_msgs::msg::Float32::SharedPtr msg)
  {
    const float d = msg->data;
    const auto r = core_.update(d);

    std_msgs::msg::Bool stop_msg;
    stop_msg.data = r.stop;
    pub_stop_->publish(stop_msg);

    // 너무 spam 되지 않게 간단 로그
    if (r.stop) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                          "STOP! dist=%.2f < %.2f (%s)", d, core_.stop_threshold(), r.reason.c_str());
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "OK dist=%.2f (thr=%.2f)", d, core_.stop_threshold());
    }
  }

  std::string topic_in_;
  std::string topic_stop_;

  autodrive_practice::SafetyFilterCore core_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyFilterNode>());
  rclcpp::shutdown();
  return 0;
}
