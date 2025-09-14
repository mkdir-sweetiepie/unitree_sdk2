/**
 * @file g1_arm_action_example.cpp
 * @brief G1 로봇 팔 동작 클라이언트를 사용하여 사전 정의된 팔 동작을 실행하는 예제
 */

// G1 로봇 팔 동작 관련 에러 정의 헤더
#include "unitree/robot/g1/arm/g1_arm_action_error.hpp"
// G1 로봇 팔 동작 클라이언트 클래스 헤더
#include "unitree/robot/g1/arm/g1_arm_action_client.hpp"

// unitree::robot::g1 네임스페이스 사용 선언
using namespace unitree::robot::g1;

int main(int argc, const char **argv)
{
    // 프로그램 시작 배너 출력
    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     G1 Arm Action Example      \n\n";

    // Unitree DDS(Data Distribution Service) 초기화
    // argv[1]이 있으면 해당 네트워크 인터페이스 사용, 없으면 기본값 "" 사용
    // 예: ./program eth0 (eth0 인터페이스 사용)
    unitree::robot::ChannelFactory::Instance()->Init(0, argc > 1 ? argv[1] : "");

    // G1 팔 동작 클라이언트 객체 생성 (스마트 포인터 사용)
    auto client = std::make_shared<unitree::robot::g1::G1ArmActionClient>();

    // 클라이언트 초기화
    client->Init();

    // 동작 실행 타임아웃을 10초로 설정
    // 모든 팔 동작은 10초 이내에 완료되어야 함
    client->SetTimeout(10.f);

    // 사용법 안내 메시지 출력
    std::cout << "Usage: \n";
    std::cout << "  - 0: print supported actions.\n"; // 0 입력: 지원되는 동작 목록 출력
    std::cout << "  - an id: execute an action.\n";   // ID 입력: 해당 동작 실행

    // 주의사항 출력
    std::cout << "Attention: \n";
    std::cout << "  Some actions will not be displayed on the APP, \n"; // 일부 동작은 앱에 표시되지 않음
    std::cout << "  but can be executed by the program.\n";             // 하지만 프로그램으로는 실행 가능
    std::cout << "  These actions may cause the robot to fall,\n";      // 이런 동작들은 로봇이 넘어질 수 있음
    std::cout << "  so please execute them with caution.\n";            // 주의해서 실행 필요

    // 동작 ID 변수 초기화
    int32_t action_id = 0;
    // 사용자 입력을 받을 문자열 변수
    std::string line;

    // 메인 실행 루프 (무한 루프)
    while (true)
    {
        // 사용자에게 동작 ID 입력 요청
        std::cout << "\nEnter action ID: .\n";

        // 한 줄 전체를 입력받음 (공백 포함)
        std::getline(std::cin, line);

        // 입력된 문자열을 정수로 변환 시도
        try
        {
            action_id = std::stoi(line); // string to integer 변환
        }
        catch (const std::exception &)
        {
            // 변환 실패 시 (숫자가 아닌 문자 입력 등) 에러 메시지 출력하고 다시 입력 받기
            std::cout << "Invalid input. Please enter an integer.\n";
            continue; // while 루프의 처음으로 돌아감
        }

        // action_id가 0인 경우: 사용 가능한 동작 목록 조회
        if (action_id == 0)
        {
            // 동작 목록을 저장할 문자열
            std::string action_list_data;

            // 서버로부터 동작 목록 가져오기
            int32_t ret = client->GetActionList(action_list_data);

            // 실패 시 에러 처리
            if (ret != 0)
            {
                std::cerr << "Failed to get action list, error code: " << ret << "\n";
                continue; // 다시 입력 받기
            }

            // 성공 시 사용 가능한 동작 목록 출력
            std::cout << "Available actions:\n"
                      << action_list_data << std::endl;
        }
        else
        { // action_id가 0이 아닌 경우: 해당 ID의 동작 실행

            // 지정된 ID의 동작 실행
            int32_t ret = client->ExecuteAction(action_id);

            // 실행 실패 시 에러 코드에 따른 처리
            if (ret != 0)
            {
                // switch문으로 에러 코드별 상세 메시지 출력
                switch (ret)
                {
                // ARM SDK 에러
                case UT_ROBOT_ARM_ACTION_ERR_ARMSDK:
                    std::cout << UT_ROBOT_ARM_ACTION_ERR_ARMSDK_DESC << std::endl;
                    break;

                // 로봇이 무언가를 잡고 있는 상태 에러
                case UT_ROBOT_ARM_ACTION_ERR_HOLDING:
                    std::cout << UT_ROBOT_ARM_ACTION_ERR_HOLDING_DESC << std::endl;
                    break;

                // 유효하지 않은 동작 ID 에러
                case UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID:
                    std::cout << UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID_DESC << std::endl;
                    break;

                // 유효하지 않은 FSM(Finite State Machine) 상태 에러
                case UT_ROBOT_ARM_ACTION_ERR_INVALID_FSM_ID:
                    // FSM ID가 500, 501, 801일 때만 동작 지원
                    std::cout << "The actions are only supported in fsm id {500, 501, 801}" << std::endl;
                    // rt/sportmodestate 토픽을 구독하여 현재 FSM ID 확인 가능
                    std::cout << "You can subscribe the topic rt/sportmodestate to check the fsm id." << std::endl;
                    // 상태 801에서는 FSM 모드가 0 또는 3일 때만 동작 지원
                    std::cout << "And in the state 801, the actions are only supported in the fsm mode {0, 3}." << std::endl;
                    // 그래도 에러가 발생하면 해당 동작은 무시
                    std::cout << "If an error is still returned at this point, ignore this action.";
                    break;

                // 기타 에러
                default:
                    std::cerr << "Execute action failed, error code: " << ret << std::endl;
                    break;
                }
            }
            // 성공 시 별도 메시지 없음 (동작이 실행됨)
        }
    }

    return 0; // 프로그램 정상 종료 (실제로는 무한 루프라 도달하지 않음)
};