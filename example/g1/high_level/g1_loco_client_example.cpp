#include <chrono>   // 시간 관련 기능 (sleep_for 등)
#include <iostream> // 입출력 스트림 (cout, cerr)
#include <thread>   // 스레드 관련 기능 (sleep)

// Unitree G1 로봇 제어 API 헤더
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

// 공백으로 구분된 문자열을 float 벡터로 변환하는 헬퍼 함수
// 예: "1.0 2.0 3.0" -> [1.0, 2.0, 3.0]
std::vector<float> stringToFloatVector(const std::string &str)
{
  std::vector<float> result; // 결과를 저장할 벡터
  std::stringstream ss(str); // 문자열을 스트림으로 변환
  float num;
  while (ss >> num)
  {                        // 스트림에서 float 값을 하나씩 읽기
    result.push_back(num); // 벡터에 추가
    ss.ignore();           // 공백 문자 무시
  }
  return result;
}

int main(int argc, char const *argv[])
{
  // 기본 설정값 - 네트워크 인터페이스는 기본적으로 "lo" (loopback)
  std::map<std::string, std::string> args = {{"network_interface", "lo"}};

  // 명령줄 인수 파싱 부분
  std::map<std::string, std::string> values;
  for (int i = 1; i < argc; ++i)
  { // argv[0]은 프로그램 이름이므로 1부터 시작
    std::string arg = argv[i];

    // "--"로 시작하는 인수만 처리
    if (arg.substr(0, 2) == "--")
    {
      size_t pos = arg.find("="); // "=" 위치 찾기
      std::string key, value;

      if (pos != std::string::npos)
      {                               // "="가 있는 경우 (예: --key=value)
        key = arg.substr(2, pos - 2); // "--" 제거하고 key 추출
        value = arg.substr(pos + 1);  // "=" 다음 부분이 value

        // 값이 따옴표로 둘러싸여 있으면 제거
        if (value.front() == '"' && value.back() == '"')
        {
          value = value.substr(1, value.length() - 2);
        }
      }
      else
      {                      // "="가 없는 경우 (예: --flag)
        key = arg.substr(2); // "--" 제거
        value = "";          // 빈 값
      }

      // args 맵에 이미 key가 있으면 업데이트, 없으면 추가
      if (args.find(key) != args.end())
      {
        args[key] = value;
      }
      else
      {
        args.insert({{key, value}});
      }
    }
  }

  // 로봇 통신 채널 초기화
  // 0: domain ID, args["network_interface"]: 네트워크 인터페이스 이름
  unitree::robot::ChannelFactory::Instance()->Init(0,
                                                   args["network_interface"]);

  // G1 로봇 제어 클라이언트 생성
  unitree::robot::g1::LocoClient client;

  // 클라이언트 초기화
  client.Init();
  // 명령 타임아웃을 10초로 설정
  client.SetTimeout(10.f);

  // 파싱된 모든 명령을 순차적으로 처리
  for (const auto &arg_pair : args)
  {
    // 현재 처리 중인 명령과 파라미터 출력
    std::cout << "Processing command: [" << arg_pair.first << "] with param: ["
              << arg_pair.second << "] ..." << std::endl;

    // network_interface는 이미 처리했으므로 건너뛰기
    if (arg_pair.first == "network_interface")
    {
      continue;
    }

    // FSM(Finite State Machine) ID 조회
    if (arg_pair.first == "get_fsm_id")
    {
      int fsm_id;
      client.GetFsmId(fsm_id); // 현재 FSM 상태 ID 가져오기
      std::cout << "current fsm_id: " << fsm_id << std::endl;
    }

    // FSM 모드 조회
    if (arg_pair.first == "get_fsm_mode")
    {
      int fsm_mode;
      client.GetFsmMode(fsm_mode); // 현재 FSM 모드 가져오기
      std::cout << "current fsm_mode: " << fsm_mode << std::endl;
    }

    // 밸런스 모드 조회
    if (arg_pair.first == "get_balance_mode")
    {
      int balance_mode;
      client.GetBalanceMode(balance_mode); // 현재 밸런스 모드 가져오기
      std::cout << "current balance_mode: " << balance_mode << std::endl;
    }

    // 발 스윙 높이 조회 (걸을 때 발을 들어올리는 높이)
    if (arg_pair.first == "get_swing_height")
    {
      float swing_height;
      client.GetSwingHeight(swing_height);
      std::cout << "current swing_height: " << swing_height << std::endl;
    }

    // 서있는 높이 조회
    if (arg_pair.first == "get_stand_height")
    {
      float stand_height;
      client.GetStandHeight(stand_height);
      std::cout << "current stand_height: " << stand_height << std::endl;
    }

    // 보행 위상(phase) 정보 조회 - 각 다리의 보행 사이클 위치
    if (arg_pair.first == "get_phase")
    {
      std::vector<float> phase;
      client.GetPhase(phase);
      std::cout << "current phase: (";
      for (const auto &p : phase)
      {
        std::cout << p << ", ";
      }
      std::cout << ")" << std::endl;
    }

    // FSM ID 설정
    if (arg_pair.first == "set_fsm_id")
    {
      int fsm_id = std::stoi(arg_pair.second); // 문자열을 정수로 변환
      client.SetFsmId(fsm_id);
      std::cout << "set fsm_id to " << fsm_id << std::endl;
    }

    // 밸런스 모드 설정
    if (arg_pair.first == "set_balance_mode")
    {
      int balance_mode = std::stoi(arg_pair.second);
      client.SetBalanceMode(balance_mode);
      std::cout << "set balance_mode to " << balance_mode << std::endl;
    }

    // 발 스윙 높이 설정
    if (arg_pair.first == "set_swing_height")
    {
      float swing_height = std::stof(arg_pair.second); // 문자열을 float로 변환
      client.SetSwingHeight(swing_height);
      std::cout << "set swing_height to " << swing_height << std::endl;
    }

    // 서있는 높이 설정
    if (arg_pair.first == "set_stand_height")
    {
      float stand_height = std::stof(arg_pair.second);
      client.SetStandHeight(stand_height);
      std::cout << "set stand_height to " << stand_height << std::endl;
    }

    // 속도 설정 (vx, vy, omega, [duration])
    if (arg_pair.first == "set_velocity")
    {
      std::vector<float> param = stringToFloatVector(arg_pair.second);
      auto param_size = param.size();
      float vx, vy, omega, duration;

      if (param_size == 3)
      {                      // 3개 파라미터: vx, vy, omega
        vx = param.at(0);    // 전진/후진 속도
        vy = param.at(1);    // 좌/우 이동 속도
        omega = param.at(2); // 회전 속도
        duration = 1.f;      // 기본 지속시간 1초
      }
      else if (param_size == 4)
      { // 4개 파라미터: duration 포함
        vx = param.at(0);
        vy = param.at(1);
        omega = param.at(2);
        duration = param.at(3); // 명령 지속시간
      }
      else
      {
        std::cerr << "Invalid param size for method SetVelocity: " << param_size
                  << std::endl;
        return 1;
      }

      client.SetVelocity(vx, vy, omega, duration);
      std::cout << "set velocity to " << arg_pair.second << std::endl;
    }

    // 댐핑 모드 - 관절에 저항을 가해 움직임을 부드럽게 정지
    if (arg_pair.first == "damp")
    {
      client.Damp();
    }

    // 로봇 시작
    if (arg_pair.first == "start")
    {
      client.Start();
    }

    // 쪼그리기 동작
    if (arg_pair.first == "squat")
    {
      client.Squat();
    }

    // 앉기 동작
    if (arg_pair.first == "sit")
    {
      client.Sit();
    }

    // 일어서기 동작
    if (arg_pair.first == "stand_up")
    {
      client.StandUp();
    }

    // 모든 관절 토크를 0으로 설정 (자유 낙하 상태)
    if (arg_pair.first == "zero_torque")
    {
      client.ZeroTorque();
    }

    // 이동 정지
    if (arg_pair.first == "stop_move")
    {
      client.StopMove();
    }

    // 높은 자세로 서기
    if (arg_pair.first == "high_stand")
    {
      client.HighStand();
    }

    // 낮은 자세로 서기
    if (arg_pair.first == "low_stand")
    {
      client.LowStand();
    }

    // 밸런스를 유지하며 서기
    if (arg_pair.first == "balance_stand")
    {
      client.BalanceStand();
    }

    // 연속 보행 모드 설정
    if (arg_pair.first == "continous_gait")
    {
      bool flag;
      if (arg_pair.second == "true")
      {
        flag = true;
      }
      else if (arg_pair.second == "false")
      {
        flag = false;
      }
      else
      {
        std::cerr << "invalid argument: " << arg_pair.second << std::endl;
        return 1;
      }
      client.ContinuousGait(flag);
    }

    // 이동 모드 전환
    if (arg_pair.first == "switch_move_mode")
    {
      bool flag;
      if (arg_pair.second == "true")
      {
        flag = true;
      }
      else if (arg_pair.second == "false")
      {
        flag = false;
      }
      else
      {
        std::cerr << "invalid argument: " << arg_pair.second << std::endl;
        return 1;
      }
      client.SwitchMoveMode(flag);
    }

    // 이동 명령 (vx, vy, omega)
    if (arg_pair.first == "move")
    {
      std::vector<float> param = stringToFloatVector(arg_pair.second);
      auto param_size = param.size();
      float vx, vy, omega;

      if (param_size == 3)
      {
        vx = param.at(0);    // 전진/후진 속도
        vy = param.at(1);    // 좌/우 이동 속도
        omega = param.at(2); // 회전 속도
      }
      else
      {
        std::cerr << "Invalid param size for method SetVelocity: " << param_size
                  << std::endl;
        return 1;
      }
      client.Move(vx, vy, omega);
    }

    // 태스크 ID 설정
    if (arg_pair.first == "set_task_id")
    {
      int task_id = std::stoi(arg_pair.second);
      client.SetTaskId(task_id);
      std::cout << "set task_id to " << task_id << std::endl;
    }

    // 악수 동작
    if (arg_pair.first == "shake_hand")
    {
      client.ShakeHand(0); // 악수 시작 (0: 시작)
      std::cout << "Shake hand starts! Waiting for 10 s for ending"
                << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(10)); // 10초 대기
      std::cout << "Shake hand ends!" << std::endl;
      client.ShakeHand(1); // 악수 종료 (1: 종료)
    }

    // 손 흔들기 동작
    if (arg_pair.first == "wave_hand")
    {
      client.WaveHand();
      std::cout << "wave hand" << std::endl;
    }

    // 몸을 돌리며 손 흔들기
    if (arg_pair.first == "wave_hand_with_turn")
    {
      client.WaveHand(true); // true: 몸 회전 포함
      std::cout << "wave hand with turn" << std::endl;
    }

    // 속도 모드 설정
    if (arg_pair.first == "set_speed_mode")
    {
      client.SetSpeedMode(std::stoi(arg_pair.second));
      std::cout << "set speed mode" << std::endl;
    }

    // 각 명령 처리 완료 메시지
    std::cout << "Done!" << std::endl;
  }

  return 0;
}