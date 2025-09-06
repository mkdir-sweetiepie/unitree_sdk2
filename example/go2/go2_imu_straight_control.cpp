/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>
#include <iostream>
#include <atomic>
#include <chrono>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

class SimpleIMUStraightController
{
public:
  SimpleIMUStraightController()
  {
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    // SportModeState만 구독하면 됨 (IMU 데이터 포함)
    state_suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    state_suber->InitChannel(std::bind(&SimpleIMUStraightController::HighStateHandler, this, std::placeholders::_1), 1);

    running = true;
    initial_yaw = 0.0f;
    target_yaw = 0.0f;
    yaw_initialized = false;

    std::cout << "간단한 IMU 직진 제어 초기화 중...\n";

    // 로봇 초기화
    for (int i = 0; i < 30; i++)
    {
      sport_client.StandUp();
      usleep(100000);
    }

    for (int i = 0; i < 30; i++)
    {
      sport_client.BalanceStand();
      usleep(100000);
    }

    sport_client.StopMove();
    std::cout << "간단한 IMU 직진 제어 준비 완료!\n";
  };

  ~SimpleIMUStraightController()
  {
    running = false;
    sport_client.StopMove();
  }

  /**
   * @brief SportModeState 처리 콜백 함수
   * @param message SportModeState 메시지
   *
   * SportModeState에는 IMU 데이터가 포함되어 있어서
   * 별도의 IMU 토픽 구독이 필요하지 않습니다.
   */
  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;

    // 최초 실행 시 초기 방향 설정 (Custom 클래스 방식과 동일)
    if (!yaw_initialized)
    {
      initial_yaw = state.imu_state().rpy()[2]; // ✅ SportModeState에서 IMU 데이터 가져오기
      target_yaw = initial_yaw;
      yaw_initialized = true;
      std::cout << "초기 Yaw 각도 설정: " << initial_yaw * 180.0f / M_PI << "도\n";
    }
  }

  /**
   * @brief 현재 Yaw 각도 반환
   * @return 현재 Yaw 각도 (라디안)
   */
  float GetCurrentYaw() const
  {
    return yaw_initialized ? state.imu_state().rpy()[2] : 0.0f;
  }

  /**
   * @brief IMU 기반 정밀 직진 이동
   * @param distance_meters 이동할 거리 (미터)
   */
  void MoveForward(float distance_meters)
  {
    if (!yaw_initialized)
    {
      std::cout << "IMU 초기화 대기 중...\n";
      return;
    }

    std::cout << "직진 시작 - 목표 거리: " << distance_meters << "m\n";

    auto start_time = std::chrono::steady_clock::now();
    float traveled_distance = 0.0f;
    const float forward_speed = 0.5f;  //3.0f; 
    const float yaw_correction_gain = 0.5f;

    while (running && traveled_distance < distance_meters)
    {
      // SportModeState에서 현재 Yaw 각도 가져오기
      float current_yaw = state.imu_state().rpy()[2]; // ✅ 간단하고 확실한 방법
      float yaw_error = target_yaw - current_yaw;

      // 각도 정규화
      while (yaw_error > M_PI)
        yaw_error -= 2 * M_PI;
      while (yaw_error < -M_PI)
        yaw_error += 2 * M_PI;

      // 방향 보정 계산
      float yaw_correction = yaw_error * yaw_correction_gain;
      yaw_correction = std::max(-0.3f, std::min(0.3f, yaw_correction));

      // 로봇 움직임 명령
      sport_client.Move(forward_speed, 0, yaw_correction);

      // 거리 계산
      auto current_time = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
      traveled_distance = forward_speed * elapsed.count() / 1000.0f;

      // 상태 출력
      std::cout << "\r직진 중... 거리: " << std::fixed << std::setprecision(2)
                << traveled_distance << "m, Yaw 오차: "
                << yaw_error * 180.0f / M_PI << "도   " << std::flush;

      usleep(20000); // 20ms 주기
    }

    sport_client.StopMove();
    std::cout << "\n직진 완료! 총 이동 거리: " << traveled_distance << "m\n";
  }

  /**
   * @brief 90도 우회전
   */
  void TurnRight90Degrees()
  {
    if (!yaw_initialized)
      return;

    std::cout << "90도 우회전 시작...\n";
    target_yaw -= M_PI / 2.0f;

    while (target_yaw < -M_PI)
      target_yaw += 2 * M_PI;

    Turn(target_yaw);
  }

  /**
   * @brief 90도 좌회전
   */
  void TurnLeft90Degrees()
  {
    if (!yaw_initialized)
      return;

    std::cout << "90도 좌회전 시작...\n";
    target_yaw += M_PI / 2.0f;

    while (target_yaw > M_PI)
      target_yaw -= 2 * M_PI;

    Turn(target_yaw);
  }

  /**
   * @brief 20도 우회전
   */
  void TurnRight20Degrees()
  {
    if (!yaw_initialized)
      return;

    std::cout << "90도 우회전 시작...\n";
    target_yaw -= M_PI / 9.0f;

    while (target_yaw < -M_PI)
      target_yaw += 2 * M_PI;

    Turn(target_yaw);
  }


  /**
   * @brief 20도 좌회전
   */
  void TurnLeft20Degrees()
  {
    if (!yaw_initialized)
      return;

    std::cout << "90도 좌회전 시작...\n";
    target_yaw += M_PI / 9.0f;

    while (target_yaw > M_PI)
      target_yaw -= 2 * M_PI;

    Turn(target_yaw);
  }

  /**
   * @brief 지정된 각도로 회전
   * @param target_angle 목표 각도
   */
  void Turn(float target_angle)
  {
    const float turn_speed = 0.5f; //3.0f;
    const float angle_tolerance = 0.05f;

    while (running)
    {
      // SportModeState에서 현재 Yaw 각도 가져오기
      float current_yaw = state.imu_state().rpy()[2]; // ✅ 간단하고 확실한 방법
      float angle_error = target_angle - current_yaw;

      // 각도 정규화
      while (angle_error > M_PI)
        angle_error -= 2 * M_PI;
      while (angle_error < -M_PI)
        angle_error += 2 * M_PI;

      // 목표 각도 도달 확인
      if (std::abs(angle_error) < angle_tolerance)
      {
        break;
      }

      // 회전 방향 결정
      float turn_direction = (angle_error > 0) ? turn_speed : -turn_speed;
      sport_client.Move(0, 0, turn_direction);

      // 상태 출력
      std::cout << "\r회전 중... 목표까지: "
                << std::fixed << std::setprecision(1)
                << angle_error * 180.0f / M_PI << "도   " << std::flush;

      usleep(20000);
    }

    sport_client.StopMove();
    std::cout << "\n회전 완료!\n";
  }

  bool IsInitialized() const
  {
    return yaw_initialized;
  }

  // 멤버 변수들
  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> state_suber;

  std::atomic<bool> running;
  float initial_yaw;
  float target_yaw;
  bool yaw_initialized;
};

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "사용법: " << argv[0] << " <network_interface>" << std::endl;
    std::cout << "예시: " << argv[0] << " enp44s0" << std::endl;
    exit(-1);
  }

  std::cout << "네트워크 인터페이스: " << argv[1] << std::endl;

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  SimpleIMUStraightController controller;

  // 충분한 초기화 시간 제공
  std::cout << "센서 초기화 대기 중...\n";
  sleep(3); // 3초 대기로 증가

  // 초기화 확인
  if (!controller.IsInitialized())
  {
    std::cout << "오류: IMU 초기화 실패!\n";
    return -1;
  }

  std::cout << "\n=== 수정된 IMU 기반 직진 제어 테스트 ===\n";
  std::cout << "SportModeState에서 IMU 데이터 직접 사용\n";
  std::cout << "1. 3미터 직진\n";
  std::cout << "2. 90도 우회전\n";
  std::cout << "3. 2미터 직진\n";
  std::cout << "4. 90도 좌회전\n";
  std::cout << "=======================================\n\n";

  // 1
  controller.MoveForward(1.0f);
  sleep(1);

  controller.TurnLeft20Degrees();
  sleep(1);

  controller.MoveForward(1.3f); // 1.0-> 1.3
  sleep(1);

  controller.TurnRight20Degrees();
  sleep(1);

  controller.MoveForward(3.2f); //3.5-> 3.2
  sleep(1);

  controller.TurnLeft90Degrees();
  sleep(1);

  controller.MoveForward(5.3f);
  sleep(1);

  controller.TurnLeft90Degrees();
  sleep(1);

  controller.MoveForward(7.5f);
  sleep(1);
  //////////////////////////////
  controller.TurnLeft90Degrees();
  sleep(1);

  controller.MoveForward(0.8f); // 1.0->0.8
  sleep(1);
  //////////////////////////////
  controller.TurnLeft90Degrees();
  sleep(1);

  controller.MoveForward(6.0f);
  sleep(1);

  controller.TurnRight90Degrees();
  sleep(1);

  controller.MoveForward(3.5f);
  sleep(1);

  controller.TurnRight90Degrees();
  sleep(1);

  controller.MoveForward(5.0f);
  sleep(1);

  std::cout << "테스트 완료!\n";
  return 0;
}