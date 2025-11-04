# ROS2 Line Tracing Autonomous Vehicle 🚗

### 📌 Overview
본 프로젝트는 **Raspberry Pi 4**와 **Arduino Mega 2560**, **파이 카메라**를 이용하여 ROS2 Humble 환경에서 동작하는  
**라인트레이싱 자율주행 차량(Line Tracing AGV)** 을 개발한 프로젝트입니다.  
카메라 영상을 실시간 처리하여 라인을 인식하고, 편차에 따라 모터를 제어하며 주행 방향을 조정합니다.  

---

### 🧩 System Architecture

- **line_camera_node.py**  
  - 파이 카메라로 촬영한 영상을 OpenCV를 통해 실시간 처리  
  - ROI(관심영역) 설정, 색상 마스크 및 이진화 수행  
  - 라인의 중심 좌표를 계산하고, 중앙과의 편차를 이용해 `/cmd_vel` 토픽으로 주행 명령 발행  

- **serial_bridge_node.py**  
  - `/cmd_vel` 토픽을 구독하여 시리얼 통신(Serial)을 통해 Arduino로 속도 명령 전달  
  - Arduino는 TB6612FNG 모터 드라이버를 통해 좌·우 모터를 제어  

- **teleop_serial_node.py**  
  - 키보드 입력(`w, a, s, d`)을 받아 `/cmd_vel` 토픽으로 전송  
  - 수동 모드(teleop)로 차량을 조작할 수 있으며, 자율주행 모드 전환도 가능  

---

### ⚙️ Hardware Components

| Component | Description |
|------------|-------------|
| Raspberry Pi 4 | ROS2 Humble 실행 (main controller) |
| Arduino Mega 2560 | 모터 제어용 시리얼 수신 컨트롤러 |
| Pi Camera | 라인 인식용 영상 입력 |
| TB6612FNG | 듀얼 DC 모터 드라이버 |
| DC Motor + Encoder | 좌/우 구동 바퀴 |
| Li-Po Battery | 이동체 전원 공급 |

---

### 🔄 Software Stack
- **ROS2 Humble**
- **Python 3.10**
- **OpenCV 4.x**
- **rclpy**, **geometry_msgs**, **cv_bridge**, **sensor_msgs**
- **Arduino C++ (Serial Communication)**

---

### 🚀 How to Run (실행 방법)

1️⃣ **빌드 및 환경 설정**
```bash
cd ~/ros_ws
colcon build
source install/setup.bash

2️⃣ 노드 개별 실행 (새 터미널 3개 띄우기)
# 터미널 1: 시리얼 브리지 노드 실행 (모터 제어)
ros2 run ros_linetracing serial_bridge_node

# 터미널 2: 카메라 인식 노드 실행 (라인 인식)
ros2 run ros_linetracing line_camera_node

# 터미널 3: 키보드 조작 노드 실행 (teleop 제어)
ros2 run ros_linetracing teleop_serial_node

3️⃣ 모드 전환
기본: teleop_serial_node를 통해 키보드 조작
자동주행: line_camera_node에서 라인 인식 후 /cmd_vel로 자동 제어
