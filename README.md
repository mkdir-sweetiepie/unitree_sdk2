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
