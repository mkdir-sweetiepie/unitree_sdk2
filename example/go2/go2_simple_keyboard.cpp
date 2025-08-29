/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>
#include <iostream>
#include <termios.h>    // 터미널 I/O 제어용 (키보드 설정)
#include <unistd.h>     // POSIX 운영체제 API (read, usleep 등)
#include <fcntl.h>      // 파일 제어 옵션 (non-blocking I/O)

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

// 로봇 상태 정보를 받아오는 토픽 정의
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

/**
 * @brief 로봇 제어 모드 열거형
 * 
 * 키보드 입력에 따른 로봇의 동작 모드를 정의합니다.
 * 각 모드는 로봇의 특정 움직임이나 자세에 대응됩니다.
 */
enum control_mode
{
  STOP = 0,        // 정지 상태
  FORWARD = 1,     // 전진
  BACKWARD = 2,    // 후진
  LEFT_TURN = 3,   // 좌회전 (제자리)
  RIGHT_TURN = 4,  // 우회전 (제자리)
  LEFT_SIDE = 5,   // 왼쪽 이동 (측면 이동)
  RIGHT_SIDE = 6,  // 오른쪽 이동 (측면 이동)
  STAND_UP = 7,    // 일어서기
  STAND_DOWN = 8   // 앉기
};

/**
 * @brief 간단한 키보드 제어 클래스
 * 
 * Unitree Go2 로봇을 실시간 키보드 입력으로 제어하는 클래스입니다.
 * 
 * 주요 기능:
 * - 키보드 raw 모드 설정 (실시간 키 입력 감지)
 * - W,A,S,D 방식의 직관적인 로봇 제어
 * - 멀티스레드 기반 실시간 제어
 * - 안전한 로봇 초기화 및 종료 처리
 */
class SimpleKeyboardController
{
public:
  /**
   * @brief 생성자 - 키보드 제어 시스템 초기화
   * 
   * 다음과 같은 초기화 과정을 수행합니다:
   * 1. Sport Client 초기화 (로봇 제어용)
   * 2. 로봇 상태 데이터 구독 설정
   * 3. 키보드 raw 모드 설정
   * 4. 로봇 물리적 초기화 (일어서기 → 균형잡기)
   * 5. 사용법 출력
   */
  SimpleKeyboardController()
  {
    // Sport Client 초기화 (로봇 움직임 제어 클라이언트)
    sport_client.SetTimeout(10.0f);  // 10초 통신 타임아웃 설정
    sport_client.Init();             // 클라이언트 초기화

    // 로봇 상태 데이터 구독자 설정
    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&SimpleKeyboardController::HighStateHandler, this, std::placeholders::_1), 1);

    // 초기 제어 모드를 정지로 설정
    current_mode = STOP;

    // 키보드를 raw 모드로 설정 (즉시 키 입력 감지용)
    InitKeyboard();

    std::cout << "로봇 초기화 중...\n";

    // 1단계: 기본 서기 자세 (Normal Stand)
    std::cout << "1/3: Normal Stand 모드...\n";
    for (int i = 0; i < 30; i++)
    {
      sport_client.StandUp();  // 일어서기 명령 반복 전송
      usleep(100000);          // 100ms 대기 (안정성 확보)
    }

    // 2단계: 균형잡힌 서기 자세 (Balance Stand)
    std::cout << "2/3: Balance Stand 모드...\n";
    for (int i = 0; i < 30; i++)
    {
      sport_client.BalanceStand(); // 균형잡기 명령 반복 전송
      usleep(100000);              // 100ms 대기
    }

    // 3단계: 안전한 정지 상태로 전환
    std::cout << "3/3: 이동 준비 완료!\n";
    sport_client.StopMove();         // 모든 움직임 정지
    std::cout << "키보드 제어 시작 가능!\n\n";

    // 사용법 도움말 출력
    PrintHelp();
  };

  /**
   * @brief 소멸자 - 시스템 종료 시 키보드 설정 복원
   * 
   * 프로그램 종료 시 터미널을 원래 설정으로 되돌립니다.
   */
  ~SimpleKeyboardController()
  {
    RestoreKeyboard();  // 키보드 설정을 원래대로 복원
  }

  /**
   * @brief 키보드 raw 모드 초기화
   * 
   * 터미널을 raw 모드로 설정하여 키를 누르자마자 즉시 감지할 수 있게 합니다.
   * 일반적으로 터미널은 Enter를 눌러야 입력을 처리하지만, 
   * raw 모드에서는 키를 누르는 순간 바로 감지됩니다.
   */
  void InitKeyboard()
  {
    // 현재 터미널 설정 백업
    tcgetattr(STDIN_FILENO, &original_termios);
    struct termios raw = original_termios;

    // raw 모드 설정
    raw.c_lflag &= ~(ICANON | ECHO);  // canonical 모드와 echo 비활성화
    raw.c_cc[VMIN] = 0;               // 최소 읽기 문자 수: 0 (즉시 반환)
    raw.c_cc[VTIME] = 0;              // 읽기 타임아웃: 0 (즉시 반환)

    // 설정 적용
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);

    // non-blocking I/O 모드 설정 (키 입력이 없어도 프로그램이 멈추지 않음)
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }

  /**
   * @brief 키보드 설정을 원래대로 복원
   * 
   * 프로그램 종료 시 터미널을 정상 모드로 되돌립니다.
   * 이를 하지 않으면 터미널이 이상하게 동작할 수 있습니다.
   */
  void RestoreKeyboard()
  {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &original_termios);
  }

  /**
   * @brief 키보드 제어 도움말 출력
   * 
   * 사용자가 로봇을 제어할 수 있는 키 조합과 기능을 출력합니다.
   * 게임의 WASD 방식을 채용하여 직관적인 제어가 가능합니다.
   */
  void PrintHelp()
  {
    std::cout << "\n========== GO2 간단 키보드 제어 ==========\n";
    std::cout << "W : 전진 (0.3 m/s)\n";
    std::cout << "S : 후진 (0.3 m/s)\n";
    std::cout << "A : 좌회전 (0.4 rad/s)\n";          // 실제 코드에서는 0.4 rad/s
    std::cout << "D : 우회전 (0.4 rad/s)\n";          // 실제 코드에서는 0.4 rad/s
    std::cout << "Q : 왼쪽 이동 (측면 이동)\n";
    std::cout << "E : 오른쪽 이동 (측면 이동)\n";
    std::cout << "R : 일어서기\n";
    std::cout << "F : 앉기\n";
    std::cout << "Space : 정지\n";
    std::cout << "ESC : 종료\n";
    std::cout << "H : 도움말 다시 보기\n";
    std::cout << "==============================\n\n";
  }

  /**
   * @brief 키보드에서 한 문자를 읽어오는 함수
   * @return 입력된 문자, 입력이 없으면 0 반환
   * 
   * non-blocking 방식으로 키보드 입력을 확인합니다.
   * 키가 눌려있으면 해당 문자를 반환하고, 없으면 0을 반환합니다.
   */
  char GetKey()
  {
    char ch;
    if (read(STDIN_FILENO, &ch, 1) == 1)  // 1바이트 읽기 시도
      return ch;                          // 성공하면 문자 반환
    return 0;                            // 실패하면 0 반환
  }

  /**
   * @brief 키보드 입력을 처리하는 함수
   * 
   * 사용자의 키 입력을 감지하고 해당하는 로봇 제어 모드로 전환합니다.
   * 각 키는 즉시 로봇의 동작 모드를 변경하며, 실제 로봇 제어는
   * 별도 스레드에서 실행되는 RobotControl() 함수에서 수행됩니다.
   */
  void HandleInput()
  {
    char key = GetKey();  // 키 입력 확인
    if (key == 0)        // 입력이 없으면 리턴
      return;

    // ESC 키 처리 (프로그램 종료)
    if (key == 27)  // ESC 키의 ASCII 코드
    { 
      std::cout << "\n프로그램 종료\n";
      running = false;  // 메인 루프 종료 플래그 설정
      return;
    }

    // 각 키에 따른 제어 모드 전환
    switch (key)
    {
    case 'w':
    case 'W':
      current_mode = FORWARD;   // 전진 모드
      std::cout << "전진\n";
      break;
      
    case 's':
    case 'S':
      current_mode = BACKWARD;  // 후진 모드
      std::cout << "후진\n";
      break;
      
    case 'a':
    case 'A':
      current_mode = LEFT_TURN; // 좌회전 모드
      std::cout << "좌회전\n";
      break;
      
    case 'd':
    case 'D':
      current_mode = RIGHT_TURN; // 우회전 모드
      std::cout << "우회전\n";
      break;
      
    case 'q':
    case 'Q':
      current_mode = LEFT_SIDE;  // 왼쪽 측면 이동
      std::cout << "왼쪽 이동\n";
      break;
      
    case 'e':
    case 'E':
      current_mode = RIGHT_SIDE; // 오른쪽 측면 이동
      std::cout << "오른쪽 이동\n";
      break;
      
    case 'r':
    case 'R':
      current_mode = STAND_UP;   // 일어서기
      std::cout << "일어서기\n";
      break;
      
    case 'f':
    case 'F':
      current_mode = STAND_DOWN; // 앉기
      std::cout << "앉기\n";
      break;
      
    case ' ':                    // 스페이스바
      current_mode = STOP;       // 정지 모드
      std::cout << "정지\n";
      break;
      
    case 'h':
    case 'H':
      PrintHelp();              // 도움말 다시 출력
      break;
    }
  }

  /**
   * @brief 실제 로봇 제어를 수행하는 함수
   * 
   * 현재 설정된 제어 모드(current_mode)에 따라 로봇에게 적절한 명령을 전송합니다.
   * 이 함수는 별도 스레드에서 10ms 주기로 반복 실행됩니다.
   * 
   * Move 함수 파라미터 설명:
   * - Move(x, y, z): x=전후, y=좌우, z=회전
   * - 양수/음수에 따라 방향이 결정됨
   * - 단위: x,y는 m/s, z는 rad/s
   */
  void RobotControl()
  {
    switch (current_mode)
    {
    case FORWARD:
      sport_client.Move(0.3, 0, 0);   // 전진: x축으로 0.3m/s
      break;

    case BACKWARD:
      sport_client.Move(-0.3, 0, 0);  // 후진: x축으로 -0.3m/s
      break;

    case LEFT_TURN:
      sport_client.Move(0, 0, 0.4);   // 좌회전: z축으로 0.4rad/s (반시계방향)
      break;

    case RIGHT_TURN:
      sport_client.Move(0, 0, -0.4);  // 우회전: z축으로 -0.4rad/s (시계방향)
      break;

    case LEFT_SIDE:
      sport_client.Move(0, 0.4, 0);   // 왼쪽 이동: y축으로 0.4m/s
      break;

    case RIGHT_SIDE:
      sport_client.Move(0, -0.4, 0);  // 오른쪽 이동: y축으로 -0.4m/s
      break;

    case STAND_UP:
      sport_client.StandUp();         // 일어서기 자세 명령
      break;

    case STAND_DOWN:
      sport_client.StandDown();       // 앉기 자세 명령
      break;

    case STOP:
    default:
      sport_client.StopMove();        // 모든 움직임 정지
      break;
    }
  }

  /**
   * @brief 로봇 상태 메시지 처리 콜백 함수
   * @param message 로봇 상태 메시지 포인터
   * 
   * 로봇으로부터 전송되는 상태 정보를 수신하여 저장합니다.
   * 현재는 단순히 데이터를 저장만 하지만, 필요에 따라
   * 로봇 상태를 모니터링하는 기능을 추가할 수 있습니다.
   */
  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;
    
    // 필요시 로봇 상태 모니터링 코드 추가 가능
    // 예: 배터리 상태, 자세 정보, 센서 데이터 등
  }

  // 클래스 멤버 변수들
  unitree_go::msg::dds_::SportModeState_ state;                                             // 로봇 상태 데이터
  unitree::robot::go2::SportClient sport_client;                                           // 로봇 제어 클라이언트
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;      // 상태 데이터 구독자

  struct termios original_termios;  // 원래 터미널 설정 백업
  control_mode current_mode;        // 현재 로봇 제어 모드
  bool running = true;              // 프로그램 실행 상태 플래그
  float dt = 0.01;                  // 제어 스레드 주기 (10ms)
};

/**
 * @brief 메인 함수 - 프로그램 진입점
 * @param argc 명령줄 인자 개수
 * @param argv 명령줄 인자 배열
 * @return 프로그램 종료 코드
 * 
 * 키보드 제어 시스템을 초기화하고 실행합니다.
 * 멀티스레드 구조로 동작하며, 한 스레드는 키보드 입력을 처리하고
 * 다른 스레드는 로봇 제어 명령을 전송합니다.
 */
int main(int argc, char **argv)
{
  // 명령줄 인자 검사 (네트워크 인터페이스 필수)
  if (argc < 2)
  {
    std::cout << "사용법: " << argv[0] << " <network_interface>" << std::endl;
    std::cout << "예시: " << argv[0] << " enp44s0" << std::endl;
    exit(-1);
  }

  std::cout << "네트워크 인터페이스: " << argv[1] << std::endl;

  // 로봇 통신 채널 초기화 (DDS 기반 실시간 통신)
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  
  // 키보드 제어기 생성 및 초기화
  SimpleKeyboardController controller;

  sleep(1);  // 초기화 완료 대기

  // 로봇 제어 스레드 생성 및 시작
  // 10ms(dt * 1000000 μs) 주기로 RobotControl() 함수를 반복 실행
  unitree::common::ThreadPtr controlThread =
      unitree::common::CreateRecurrentThread(controller.dt * 1000000,
                                             std::bind(&SimpleKeyboardController::RobotControl, &controller));

  // 메인 스레드에서 키보드 입력 처리
  // 사용자가 ESC를 누르거나 프로그램 종료까지 계속 실행
  while (controller.running)
  {
    controller.HandleInput();  // 키보드 입력 확인 및 처리
    usleep(50000);            // 50ms 대기 (CPU 사용률 최적화)
  }

  std::cout << "프로그램 종료\n";
  return 0;
}