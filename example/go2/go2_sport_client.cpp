/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

enum test_mode
{
  /*---Basic motion---*/
  normal_stand = 0,  // [수정] 명시적으로 0부터 시작하도록 값 지정
  balance_stand = 1,
  velocity_move = 2,
  stand_down = 3,
  stand_up = 4,
  damp = 5,
  recovery_stand = 6,
  /*---Special motion ---*/
  sit = 7,
  rise_sit = 8,
  stop_move = 99
};

// [삭제] const int TEST_MODE = stand_down; 를 제거하고 main에서 인자로 받도록 변경

class Custom
{
public:
  int TEST_MODE = normal_stand;  // [추가] 클래스 멤버 변수로 TEST_MODE 추가
  
  Custom()
  {
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
  };

  // [추가] TEST_MODE를 설정하는 함수
  void SetTestMode(int mode)
  {
    TEST_MODE = mode;
    std::cout << "Mode set to: " << mode << std::endl;
  }

  void RobotControl()
  {
    ct += dt;
    double px_local, py_local, yaw_local;
    double vx_local, vy_local, vyaw_local;
    double px_err, py_err, yaw_err;
    double time_seg, time_temp;

    unitree::robot::go2::PathPoint path_point_tmp;
    std::vector<unitree::robot::go2::PathPoint> path;

    switch (TEST_MODE)
    {
    case normal_stand:            // 0. idle, default stand
      // sport_client.SwitchGait(0); // 0:idle; 1:tort; 2:tort running; 3:climb stair; 4:tort obstacle
      sport_client.StandUp();
      break;

    case balance_stand:                  // 1. Balance stand (controlled by dBodyHeight + rpy)
      // sport_client.Euler(0.1, 0.2, 0.3); // roll, pitch, yaw
      // sport_client.BodyHeight(0.0);      // relative height [-0.18~0.03]
      sport_client.BalanceStand();
      break;

    case velocity_move: // 2. target velocity walking (controlled by velocity + yawSpeed)
      sport_client.Move(0.3, 0, 0.3);
      break;

    case stand_down: // 3. position stand down.  [수정] 주석 번호 변경
      sport_client.StandDown();
      break;

    case stand_up: // 4. position stand up  [수정] 주석 번호 변경
      sport_client.StandUp();
      break;

    case damp: // 5. damping mode  [수정] 주석 번호 변경
      sport_client.Damp();
      break;

    case recovery_stand: // 6. recovery stand  [수정] 주석 번호 변경
      sport_client.RecoveryStand();
      break;

    case sit:  // 7. sit  [추가] 주석 번호 추가
      if (flag == 0)
      {
        sport_client.Sit();
        flag = 1;
      }
      break;

    case rise_sit:  // 8. rise sit  [추가] 주석 번호 추가
      if (flag == 0)
      {
        sport_client.RiseSit();
        flag = 1;
      }
      break;

    case stop_move: // 99. stop move  [수정] 주석 번호 추가
      sport_client.StopMove();
      break;

    default:
      sport_client.StopMove();
    }
  };

  // Get initial position
  void GetInitState()
  {
    px0 = state.position()[0];
    py0 = state.position()[1];
    yaw0 = state.imu_state().rpy()[2];
    std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;
  };

  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;

    // std::cout << "Position: " << state.position()[0] << ", " << state.position()[1] << ", " << state.position()[2] << std::endl;
    // std::cout << "IMU rpy: " << state.imu_state().rpy()[0] << ", " << state.imu_state().rpy()[1] << ", " << state.imu_state().rpy()[2] << std::endl;
  };

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

  double px0, py0, yaw0; // 초기 상태의 위치와 偏航
  double ct = 0;         // 실행 시간
  int flag = 0;          // 특수 동작 실행 플래그
  float dt = 0.005;      // 제어 스텝 0.001~0.01
};

// [추가] 사용법을 출력하는 함수
void PrintUsage(const char* programName)
{
  std::cout << "Usage: " << programName << " <network_interface> <mode>" << std::endl;
  std::cout << "\nAvailable modes:" << std::endl;
  std::cout << "  0 : normal_stand (기본 서기)" << std::endl;
  std::cout << "  1 : balance_stand (균형 서기)" << std::endl;
  std::cout << "  2 : velocity_move (이동 - 전진 0.3m/s, 회전 0.3rad/s)" << std::endl;
  std::cout << "  3 : stand_down (앉기)" << std::endl;
  std::cout << "  4 : stand_up (일어서기)" << std::endl;
  std::cout << "  5 : damp (댐핑 모드)" << std::endl;
  std::cout << "  6 : recovery_stand (복구 서기)" << std::endl;
  std::cout << "  7 : sit (앉기 자세)" << std::endl;
  std::cout << "  8 : rise_sit (앉은 자세에서 일어나기)" << std::endl;
  std::cout << "  99: stop_move (정지)" << std::endl;
  std::cout << "\nExample:" << std::endl;
  std::cout << "  " << programName << " enp44s0 0    # normal stand mode" << std::endl;
  std::cout << "  " << programName << " enp44s0 2    # velocity move mode" << std::endl;
  std::cout << "  " << programName << " enp44s0 3    # stand down mode" << std::endl;
}

int main(int argc, char **argv)
{
  // [수정] 인자 개수를 2개에서 3개로 변경 (프로그램명, 네트워크인터페이스, 모드)
  if (argc < 3)
  {
    PrintUsage(argv[0]);  // [추가] 사용법 출력
    exit(-1);
  }

  // [추가] 모드 값 파싱 및 유효성 검사
  int mode;
  try {
    mode = std::stoi(argv[2]);
  } catch (const std::exception& e) {
    std::cerr << "Error: Invalid mode value. Please enter a number." << std::endl;
    PrintUsage(argv[0]);
    exit(-1);
  }

  // [추가] 모드 값 범위 확인
  if (mode < 0 || (mode > 8 && mode != 99)) {
    std::cerr << "Error: Invalid mode value. Mode must be 0-8 or 99." << std::endl;
    PrintUsage(argv[0]);
    exit(-1);
  }

  // [추가] 선택된 모드 출력
  std::cout << "Starting with network interface: " << argv[1] << std::endl;
  std::cout << "Selected mode: " << mode;
  switch(mode) {
    case 0: std::cout << " (normal_stand)"; break;
    case 1: std::cout << " (balance_stand)"; break;
    case 2: std::cout << " (velocity_move)"; break;
    case 3: std::cout << " (stand_down)"; break;
    case 4: std::cout << " (stand_up)"; break;
    case 5: std::cout << " (damp)"; break;
    case 6: std::cout << " (recovery_stand)"; break;
    case 7: std::cout << " (sit)"; break;
    case 8: std::cout << " (rise_sit)"; break;
    case 99: std::cout << " (stop_move)"; break;
  }
  std::cout << std::endl << std::endl;

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  Custom custom;
  
  custom.SetTestMode(mode);  // [추가] 모드 설정

  sleep(1); // Wait for 1 second to obtain a stable state

  custom.GetInitState(); // Get initial position
  unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::RobotControl, &custom));

  while (1)
  {
    sleep(10);
  }
  return 0;
}