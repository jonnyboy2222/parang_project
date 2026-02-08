#include <chrono> // 시간 라이브러리, 타이머 주기나 시간 간격 표현
#include <random> // 난수 생성 라이브러리
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp> // ros 표준 메세지 타입

using namespace std::chrono_literals;

class FakeLidarNode : public rclcpp::Node // rclcpp::Node를 상속 (클래스 자체가 ROS2 노드)
{
public: // 외부에서 접근 가능
  FakeLidarNode() : Node("fake_lidar") // 노드 이름으로 생성자 호출
  {
    // parameters
    // ROS2 파라미터는 반드시 declare 먼저
    // 여기서 this는 FakeLidarNode 객체를 가리키는 포인터
    this->declare_parameter<std::string>("topic_out", "/fake/lidar_distance_m");
    this->declare_parameter<double>("rate_hz", 10.0);
    this->declare_parameter<double>("min_m", 0.5);
    this->declare_parameter<double>("max_m", 20.0);

    // parameter서버에서 값 읽어들이기
    topic_out_ = this->get_parameter("topic_out").as_string();
    rate_hz_   = this->get_parameter("rate_hz").as_double();
    min_m_     = this->get_parameter("min_m").as_double();
    max_m_     = this->get_parameter("max_m").as_double();

    // publisher 생성 create_publisher<메세지 타입>(토픽이름, QoS)
    // QoS(10) : 버퍼에 최대 10개 메세지 저장
    pub_ = this->create_publisher<std_msgs::msg::Float32>(topic_out_, rclcpp::QoS(10));

    // random
    std::random_device rd;
    rng_ = std::mt19937(rd());
    // min과 max 사이의 균등분포
    dist_ = std::uniform_real_distribution<float>(static_cast<float>(min_m_), static_cast<float>(max_m_));

    auto period = std::chrono::duration<double>(1.0 / std::max(0.1, rate_hz_));
    timer_ = this->create_wall_timer( // (주기, 콜백함수)
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&FakeLidarNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "Publishing fake distance to %s (%.2f Hz, range %.2f~%.2f m)",
                topic_out_.c_str(), rate_hz_, min_m_, max_m_);
  }
// std::는 c++ 표준 라이브러리 표시
// 다른 메서드와 혼동하지 않기 위함
private:
  void on_timer()
  {
    std_msgs::msg::Float32 msg;
    msg.data = dist_(rng_);
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string topic_out_;
  double rate_hz_{10.0};
  double min_m_{0.5};
  double max_m_{20.0};

  std::mt19937 rng_;
  std::uniform_real_distribution<float> dist_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeLidarNode>());
  rclcpp::shutdown();
  return 0;
}
