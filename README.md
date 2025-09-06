# unitree_sdk2
Unitree robot sdk version 2.

### Prebuild environment
* OS  (Ubuntu 20.04 LTS)  
* CPU  (aarch64 and x86_64)   
* Compiler  (gcc version 9.4.0) 

### Build examples

To build the examples inside this repository:

```bash
mkdir build
cd build
cmake ..
make
```

### Installation

To build your own application with the SDK, you can install the unitree_sdk2 to your system directory:

```bash
mkdir build
cd build
cmake ..
sudo make install
```

Or install unitree_sdk2 to a specified directory:

```bash
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

You can refer to `example/cmake_sample` on how to import the unitree_sdk2 into your CMake project. 

Note that if you install the library to other places other than `/opt/unitree_robotics`, you need to make sure the path is added to "${CMAKE_PREFIX_PATH}" so that cmake can find it with "find_package()".

### Notice
For more reference information, please go to [Unitree Document Center](https://support.unitree.com/home/zh/developer).


---
```
hyeon@hyeon ~/g1/unitree_sdk2/build/bin (main) $ ./go2_sport_client 
Usage: ./go2_sport_client <network_interface> <mode>

Available modes:
  0 : normal_stand (기본 서기)
  1 : balance_stand (균형 서기)
  2 : velocity_move (이동 - 전진 0.3m/s, 회전 0.3rad/s)
  3 : stand_down (앉기)
  4 : stand_up (일어서기)
  5 : damp (댐핑 모드)
  6 : recovery_stand (복구 서기)
  7 : sit (앉기 자세)
  8 : rise_sit (앉은 자세에서 일어나기)
  99: stop_move (정지)

Example:
  ./go2_sport_client enp44s0 0    # normal stand mode
  ./go2_sport_client enp44s0 2    # velocity move mode
  ./go2_sport_client enp44s0 3    # stand down mode
```

```
./go2_sport_client enp44s0 0
./go2_sport_client enp44s0 1
./go2_sport_client enp44s0 2
./go2_sport_client enp44s0 5
```

```
cd ~/unitree_sdk2/
rm -rf build
mkdir build
cd build
cmake ..
make
cd bin
./go2_imu_straight_control enp44s0
```

```
hyeon@hyeon:~/unitree_sdk2/build/bin$ ./go2_imu_straight_control enp44s0
네트워크 인터페이스: enp44s0
간단한 IMU 직진 제어 초기화 중...
초기 Yaw 각도 설정: 159.277도
간단한 IMU 직진 제어 준비 완료!
센서 초기화 대기 중...

=== 수정된 IMU 기반 직진 제어 테스트 ===
SportModeState에서 IMU 데이터 직접 사용
1. 3미터 직진
2. 90도 우회전
3. 2미터 직진
4. 90도 좌회전
=======================================

직진 시작 - 목표 거리: 1m
직진 중... 거리: 1.00m, Yaw 오차: -4.49도   
직진 완료! 총 이동 거리: 1.00m
90도 좌회전 시작...
회전 중... 목표까지: 3.1도    
회전 완료!
직진 시작 - 목표 거리: 1.0m
직진 중... 거리: 1.00m, Yaw 오차: -0.53도   
직진 완료! 총 이동 거리: 1.00m
90도 우회전 시작...
회전 중... 목표까지: -2.9도    
회전 완료!
직진 시작 - 목표 거리: 3.5m
직진 중... 거리: 3.50m, Yaw 오차: 7.47도    
직진 완료! 총 이동 거리: 3.50m
90도 좌회전 시작...
회전 중... 목표까지: 80.5도 
```
