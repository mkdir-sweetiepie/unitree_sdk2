이 코드는 Unitree G1 로봇의 팔을 저수준(Low - level) 에서 직접 제어하는 예제입니다.한줄한줄 주석을 추가하여 설명해드리겠습니다 : cpp #include<array> // std::array 사용
#include <chrono>                                                                                                                                    // 시간 관련 기능
#include <iostream>                                                                                                                                  // 입출력
#include <thread>                                                                                                                                    // 스레드 슬립 기능

// Unitree 메시지 타입 헤더
#include <unitree/idl/hg/LowCmd_.hpp>                   // 저수준 명령 메시지
#include <unitree/idl/hg/LowState_.hpp>                 // 저수준 상태 메시지
#include <unitree/robot/channel/channel_publisher.hpp>  // DDS 퍼블리셔
#include <unitree/robot/channel/channel_subscriber.hpp> // DDS 서브스크라이버

                                                                                                                                 // DDS 토픽 이름 정의
                                                                                                                                 static const std::string kTopicArmSDK = "rt/arm_sdk"; // 팔 제어 명령 토픽
static const std::string kTopicState = "rt/lowstate";                                                                                                                                  // 로봇 상태 토픽

// 수학 상수 정의
constexpr float kPi = 3.141592654;  // 파이
constexpr float kPi_2 = 1.57079632; // 파이/2 (90도)

// 로봇의 모든 관절 인덱스를 열거형으로 정의
enum JointIndex
{
  // 왼쪽 다리 관절 (6개)
  kLeftHipPitch,  // 왼쪽 엉덩이 피치
  kLeftHipRoll,   // 왼쪽 엉덩이 롤
  kLeftHipYaw,    // 왼쪽 엉덩이 요
  kLeftKnee,      // 왼쪽 무릎
  kLeftAnkle,     // 왼쪽 발목
  kLeftAnkleRoll, // 왼쪽 발목 롤

  // 오른쪽 다리 관절 (6개)
  kRightHipPitch,  // 오른쪽 엉덩이 피치
  kRightHipRoll,   // 오른쪽 엉덩이 롤
  kRightHipYaw,    // 오른쪽 엉덩이 요
  kRightKnee,      // 오른쪽 무릎
  kRightAnkle,     // 오른쪽 발목
  kRightAnkleRoll, // 오른쪽 발목 롤

  // 허리 관절 (3개)
  kWaistYaw,   // 허리 요
  kWaistRoll,  // 허리 롤
  kWaistPitch, // 허리 피치

  // 왼쪽 팔 관절 (7개)
  kLeftShoulderPitch, // 왼쪽 어깨 피치
  kLeftShoulderRoll,  // 왼쪽 어깨 롤
  kLeftShoulderYaw,   // 왼쪽 어깨 요
  kLeftElbow,         // 왼쪽 팔꿈치
  kLeftWristRoll,     // 왼쪽 손목 롤
  kLeftWristPitch,    // 왼쪽 손목 피치
  kLeftWristYaw,      // 왼쪽 손목 요

  // 오른쪽 팔 관절 (7개)
  kRightShoulderPitch, // 오른쪽 어깨 피치
  kRightShoulderRoll,  // 오른쪽 어깨 롤
  kRightShoulderYaw,   // 오른쪽 어깨 요
  kRightElbow,         // 오른쪽 팔꿈치
  kRightWristRoll,     // 오른쪽 손목 롤
  kRightWristPitch,    // 오른쪽 손목 피치
  kRightWristYaw,      // 오른쪽 손목 요

  // 사용하지 않는 관절 인덱스 (예비)
  kNotUsedJoint, // weight 값 전달용으로 사용
  kNotUsedJoint1,
  kNotUsedJoint2,
  kNotUsedJoint3,
  kNotUsedJoint4,
  kNotUsedJoint5
};

int main(int argc, char const *argv[])
{
  // 네트워크 인터페이스 인자 확인
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  // DDS 통신 채널 초기화 (0: domain ID, argv[1]: 네트워크 인터페이스)
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  // 팔 제어 명령을 발행할 퍼블리셔 생성
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>
      arm_sdk_publisher;
  unitree_hg::msg::dds_::LowCmd_ msg; // 명령 메시지 객체

  // 퍼블리셔 초기화 (rt/arm_sdk 토픽으로 발행)
  arm_sdk_publisher.reset(
      new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(
          kTopicArmSDK));
  arm_sdk_publisher->InitChannel();

  // 로봇 상태를 수신할 서브스크라이버 생성
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>
      low_state_subscriber;

  // 상태 메시지 객체와 서브스크라이버 초기화
  unitree_hg::msg::dds_::LowState_ state_msg;
  low_state_subscriber.reset(
      new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
          kTopicState));

  // 콜백 함수 등록 - 상태 메시지 수신 시 state_msg에 복사
  low_state_subscriber->InitChannel([&](const void *msg)
                                    {
        auto s = ( const unitree_hg::msg::dds_::LowState_* )msg;
        memcpy( &state_msg, s, sizeof( unitree_hg::msg::dds_::LowState_ ) ); }, 1); // 1: 버퍼 크기

  // 제어할 팔 관절 인덱스 배열 (왼팔 7개 + 오른팔 7개 + 허리 3개 = 17개)
  std::array<JointIndex, 17> arm_joints = {
      // 왼팔 관절들
      JointIndex::kLeftShoulderPitch, JointIndex::kLeftShoulderRoll,
      JointIndex::kLeftShoulderYaw, JointIndex::kLeftElbow,
      JointIndex::kLeftWristRoll, JointIndex::kLeftWristPitch,
      JointIndex::kLeftWristYaw,
      // 오른팔 관절들
      JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
      JointIndex::kRightShoulderYaw, JointIndex::kRightElbow,
      JointIndex::kRightWristRoll, JointIndex::kRightWristPitch,
      JointIndex::kRightWristYaw,
      // 허리 관절들
      JointIndex::kWaistYaw,
      JointIndex::kWaistRoll,
      JointIndex::kWaistPitch};

  // 제어 가중치 (0: 제어 안함, 1: 완전 제어)
  float weight = 0.f;
  float weight_rate = 0.2f; // 가중치 변화율

  // PD 제어 게인값
  float kp = 60.f;    // 비례 게인 (위치 제어)
  float kd = 1.5f;    // 미분 게인 (속도 제어/댐핑)
  float dq = 0.f;     // 목표 관절 속도 (0으로 설정)
  float tau_ff = 0.f; // 피드포워드 토크 (0으로 설정)

  // 제어 파라미터
  float control_dt = 0.02f;        // 제어 주기 (20ms = 50Hz)
  float max_joint_velocity = 0.5f; // 최대 관절 속도 (rad/s)

  // 계산된 파라미터
  float delta_weight = weight_rate * control_dt;           // 한 스텝당 가중치 변화량
  float max_joint_delta = max_joint_velocity * control_dt; // 한 스텝당 최대 관절 변화량

  // 제어 주기에 맞는 슬립 시간 (밀리초 단위)
  auto sleep_time =
      std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));

  // 초기 자세 (모든 관절 0도 - 팔을 내린 자세)
  std::array<float, 17> init_pos{0, 0, 0, 0, 0, 0, 0, // 왼팔
                                 0, 0, 0, 0, 0, 0, 0, // 오른팔
                                 0, 0, 0};            // 허리

  // 목표 자세 (팔을 들어올린 자세)
  // 왼팔: 어깨 롤 90도, 팔꿈치 90도
  // 오른팔: 어깨 롤 -90도, 팔꿈치 90도
  std::array<float, 17> target_pos = {0.f, kPi_2, 0.f, kPi_2, 0.f, 0.f, 0.f,  // 왼팔
                                      0.f, -kPi_2, 0.f, kPi_2, 0.f, 0.f, 0.f, // 오른팔
                                      0, 0, 0};                               // 허리

  // 초기화 대기
  std::cout << "Press ENTER to init arms ...";
  std::cin.get(); // 엔터 키 입력 대기

  // 현재 관절 위치 읽기
  std::array<float, 17> current_jpos{};
  std::cout << "Current joint position: ";
  for (int i = 0; i < arm_joints.size(); ++i)
  {
    // 각 팔 관절의 현재 위치(q) 저장
    current_jpos.at(i) = state_msg.motor_state().at(arm_joints.at(i)).q();
    std::cout << current_jpos.at(i) << " ";
  }
  std::cout << std::endl;

  // 초기 자세로 이동 (2초간 부드럽게 전환)
  std::cout << "Initailizing arms ...";
  float init_time = 2.0f;                                         // 초기화 시간
  int init_time_steps = static_cast<int>(init_time / control_dt); // 필요한 스텝 수

  for (int i = 0; i < init_time_steps; ++i)
  {
    // 가중치를 1로 설정 (완전 제어)
    weight = 1.0;
    msg.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);

    // 보간 비율 계산 (0 -> 1)
    float phase = 1.0 * i / init_time_steps;
    std::cout << "Phase: " << phase << std::endl;

    // 각 관절에 대해 현재 위치에서 초기 위치로 선형 보간
    for (int j = 0; j < init_pos.size(); ++j)
    {
      // 보간된 목표 위치 = 초기위치 * phase + 현재위치 * (1-phase)
      msg.motor_cmd().at(arm_joints.at(j)).q(init_pos.at(j) * phase + current_jpos.at(j) * (1 - phase));
      msg.motor_cmd().at(arm_joints.at(j)).dq(dq);      // 목표 속도
      msg.motor_cmd().at(arm_joints.at(j)).kp(kp);      // P 게인
      msg.motor_cmd().at(arm_joints.at(j)).kd(kd);      // D 게인
      msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff); // 피드포워드 토크
    }

    // DDS 메시지 발행
    arm_sdk_publisher->Write(msg);

    // 제어 주기만큼 대기
    std::this_thread::sleep_for(sleep_time);
  }

  std::cout << "Done!" << std::endl;

  // 제어 시작 대기
  std::cout << "Press ENTER to start arm ctrl ..." << std::endl;
  std::cin.get();

  // 팔 제어 시작
  std::cout << "Start arm ctrl!" << std::endl;
  float period = 5.f;                                         // 동작 시간 (5초)
  int num_time_steps = static_cast<int>(period / control_dt); // 필요한 스텝 수

  std::array<float, 17> current_jpos_des{}; // 현재 목표 위치

  // 팔 들어올리기 (5초간)
  for (int i = 0; i < num_time_steps; ++i)
  {
    // 각 관절의 목표 위치를 점진적으로 업데이트
    for (int j = 0; j < init_pos.size(); ++j)
    {
      // 목표까지 남은 거리를 계산하고, 최대 속도로 제한
      current_jpos_des.at(j) +=
          std::clamp(target_pos.at(j) - current_jpos_des.at(j), // 남은 거리
                     -max_joint_delta, max_joint_delta);        // 속도 제한
    }

    // 각 관절에 명령 설정
    for (int j = 0; j < init_pos.size(); ++j)
    {
      msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos_des.at(j));
      msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
      msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
      msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }

    // DDS 메시지 발행
    arm_sdk_publisher->Write(msg);

    // 제어 주기만큼 대기
    std::this_thread::sleep_for(sleep_time);
  }

  // 팔 내리기 (5초간) - 초기 자세로 복귀
  for (int i = 0; i < num_time_steps; ++i)
  {
    // 각 관절을 초기 위치로 점진적으로 이동
    for (int j = 0; j < init_pos.size(); ++j)
    {
      current_jpos_des.at(j) +=
          std::clamp(init_pos.at(j) - current_jpos_des.at(j), // 초기 위치까지 거리
                     -max_joint_delta, max_joint_delta);      // 속도 제한
    }

    // 각 관절에 명령 설정
    for (int j = 0; j < init_pos.size(); ++j)
    {
      msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos_des.at(j));
      msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
      msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
      msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }

    // DDS 메시지 발행
    arm_sdk_publisher->Write(msg);

    // 제어 주기만큼 대기
    std::this_thread::sleep_for(sleep_time);
  }

  // 제어 종료 (2초간 가중치를 점진적으로 감소)
  std::cout << "Stoping arm ctrl ...";
  float stop_time = 2.0f; // 종료 시간
  int stop_time_steps = static_cast<int>(stop_time / control_dt);

  for (int i = 0; i < stop_time_steps; ++i)
  {
    // 가중치를 점진적으로 감소 (1 -> 0)
    weight -= delta_weight;
    weight = std::clamp(weight, 0.f, 1.f); // 0~1 범위로 제한

    // 가중치 설정 (NotUsedJoint 인덱스 사용)
    msg.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);

    // DDS 메시지 발행
    arm_sdk_publisher->Write(msg);

    // 제어 주기만큼 대기
    std::this_thread::sleep_for(sleep_time);
  }

  // 최종적으로 가중치를 0으로 설정 (제어 완전 해제)
  msg.motor_cmd().at(JointIndex::kNotUsedJoint).q(0);

  // 마지막 메시지 발행
  arm_sdk_publisher->Write(msg);

  std::cout << "Done!" << std::endl;

  return 0;
}