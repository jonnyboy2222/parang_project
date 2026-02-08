#pragma once // 헤더 중복 include 방지
#include <string>

namespace autodrive_practice // 이름 충돌 방지
{
struct SafetyResult // 결과를 담을 구조체
{
  bool stop{false}; // 중괄호 내 값으로 초기화
  std::string reason; // 빈 문자열 (기본값 x)
};

class SafetyFilterCore
{
public: // public 정의는 외부에서 호출 가능
  explicit SafetyFilterCore(float stop_threshold_m = 2.0f) // 생성자, explicit으로 암묵적 형변환 막음
  : stop_threshold_m_(stop_threshold_m) {} // 클래스 멤버 변수를 생성자 인자로 초기화, {}은 생성자 실행 코드, 지금은 빈 본문 (초기화만 하고 끝)

  void set_stop_threshold(float v) { stop_threshold_m_ = v; } // setter: threshold 값을 바꿈 (파라미터 변경 콜백)
  float stop_threshold() const { return stop_threshold_m_; } // getter: const는 멤버를 변경하지 않는다는 약속
  // 밖에서 값을 호출할 수 있도록 함

  // 입력(distance_m)을 보고 정지해야 하는지 판단하는 함수 update
  SafetyResult update(float distance_m) const // 반환 타입이 SafetyResult 타입, const는 내부 상태를 바꾸지 않는다는 선언
  {
    SafetyResult r; // 객체 생성
    if (distance_m < stop_threshold_m_) {
      r.stop = true;
      r.reason = "Obstacle too close";
    } else {
      r.stop = false;
      r.reason = "Safe";
    }
    return r;
  }

private:
  float stop_threshold_m_; // 끝에 _를 붙이는건 멤버변수라는 표시 관례
};
}  // namespace autodrive_practice
