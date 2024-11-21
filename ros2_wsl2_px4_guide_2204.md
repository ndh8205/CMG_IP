# WSL2 Ubuntu 22.04 ROS2 Humble/PX4/Gazebo 시뮬레이션 환경 구축 가이드

## 목차

### Part 1: 기본 환경 설정 및 ROS2 설치
1. WSL2 환경 설정
2. 기본 패키지 설치
3. ROS2 Humble 설치
4. ROS2 개발 도구 설치
5. Gazebo Classic 설치

### Part 2: PX4 및 DDS 설치
6. PX4 설치
7. DDS 설치
8. ROS2 작업공간 설정
9. 최종 환경 변수 설정
10. 시뮬레이션 실행 및 테스트

## 1. WSL2 환경 설정

```bash
# 현재 설치된 WSL 배포판 목록 확인
wsl --list -v

# 기존 Ubuntu 20.04 종료 및 제거 (필요한 경우)
wsl --terminate Ubuntu-22.04    # 실행 중인 WSL 인스턴스 종료
wsl --unregister Ubuntu-22.04   # WSL 배포판 완전 제거 (모든 데이터 삭제됨)

# 새로운 Ubuntu 22.04 설치
wsl --install -d Ubuntu-22.04

# 설치 후 Ubuntu 버전 확인
lsb_release -a
```

## 2. 기본 패키지 설치

```bash
# 시스템 업데이트
sudo apt update && sudo apt upgrade -y

# 필수 패키지 설치
sudo apt install -y \
    git \
    build-essential \
    cmake \
    libssl-dev \
    wget \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    ninja-build \
    exiftool \
    astyle \
    ccache \
    clang \
    clang-tidy \
    g++ \
    gcc \
    gdb \
    make \
    rsync \
    shellcheck \
    unzip \
    xsltproc \
    zip

# Java 설치
sudo apt install -y default-jdk
echo "export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64" >> ~/.bashrc
source ~/.bashrc

# libasio 설치
sudo apt install -y libasio-dev
```

## 3. ROS2 Humble 설치

```bash
# locale 설정
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 저장소 추가
sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble 설치
sudo apt update
sudo apt install -y ros-humble-desktop
```

## 4. ROS2 개발 도구 설치

```bash
# ROS2 개발 도구 및 의존성 패키지 설치
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-flake8 \
    python3-pytest-cov \
    python3-setuptools \
    python3-vcstool \
    python3-rosinstall-generator \
    ros-humble-ament-* \
    ros-humble-ros-testing \
    ros-humble-eigen3-cmake-module

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# rosdep 초기화 및 업데이트
sudo rosdep init || true
rosdep update

# colcon mixin 추가
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update
```

## 5. Gazebo Classic 설치

```bash
# Gazebo Classic 설치 (PX4 v1.16.0에 필요)
sudo apt-get update
sudo apt-get install -y gazebo libgazebo-dev

# ROS2-Gazebo 인터페이스 패키지 설치
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces \
    ros-humble-ros-gz-sim

# 추가 의존성 패키지 설치
sudo apt install -y \
    python3-jinja2 \
    python3-pip \
    python3-cerberus \
    python3-numpy \
    python3-yaml \
    python3-setuptools \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly
```

## 6. PX4 설치

```bash
# PX4 Firmware 클론
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# PX4 툴체인 설치
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh

# 환경 다시 불러오기
source ~/.bashrc
```

## 7. DDS 설치

```bash
# 1. 기존 설치 제거 (있다면)
cd ~
sudo rm -rf Fast-CDR fastcdr Fast-DDS Fast-DDS-Gen
sudo rm -rf /usr/local/include/fastcdr /usr/local/include/fastrtps
sudo rm -rf /usr/local/lib/cmake/fastcdr /usr/local/lib/cmake/fastrtps
sudo ldconfig

# 2. FastCDR v1.0.27 설치 (Fast-DDS 2.10.1과 호환)
git clone https://github.com/eProsima/Fast-CDR.git fastcdr
cd fastcdr
git checkout v1.0.27
mkdir build && cd build
cmake ..
make
sudo cmake --build . --target install
sudo ldconfig

# 3. Fast-DDS 2.10.1 설치
cd ~
git clone --recursive https://github.com/eProsima/Fast-DDS.git
cd Fast-DDS
git checkout v2.10.1
git submodule update --init --recursive

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DTHIRDPARTY=ON -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig

# 4. Fast-DDS-Gen 설치
cd ~
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v2.4.0
cd Fast-DDS-Gen
./gradlew assemble
sudo ./gradlew install

# 5. Micro-XRCE-DDS-Agent 설치
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 실행 파일을 시스템 경로에 복사
sudo cp MicroXRCEAgent /usr/local/bin/
sudo ldconfig

# 환경 변수 설정
echo 'export PATH=$PATH:/usr/local/bin' >> ~/.bashrc
source ~/.bashrc
```

## 8. ROS2 작업공간 설정

```bash
# ROS2 workspace 생성
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# PX4 ROS2 패키지 설치
cd src
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git

# workspace 루트 디렉토리로 이동
cd ~/ros2_ws

# ROS2 환경 설정 (이미 다른 터미널에서 했더라도 다시 실행)
source /opt/ros/humble/setup.bash

# workspace 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# workspace 빌드
colcon build --packages-select px4_msgs px4_ros_com

# 새로 빌드한 패키지 환경 설정 적용
source install/setup.bash
```

## 9. 최종 환경 변수 설정

```bash
# 환경 변수 설정 추가
cat << 'EOL' >> ~/.bashrc

# ROS2 및 Colcon 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Gazebo 설정
source /usr/share/gazebo/setup.sh

# PX4 설정
source ~/PX4-Autopilot/Tools/simulation/gazebo/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo

# DDS 설정
export PATH=$PATH:/usr/local/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
EOL

# 새 환경 변수 적용
source ~/.bashrc
```

## 10. 시뮬레이션 실행 및 테스트

터미널 1: PX4 SITL 실행
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

터미널 2: DDS Agent 실행 (터미널 1의 실행이 완료된 후 실행)
```bash
# 환경 설정 로드
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

MicroXRCEAgent udp4 -p 8888
```

터미널 3: ROS2 노드 실행 (DDS Agent 실행 후)
```bash
# 환경 설정 로드
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd ~/ros2_ws
ros2 run px4_ros_com sensor_combined_listener
```
