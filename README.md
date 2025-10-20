# 🏎️ F1TENTH Gym ROS2 Communication Bridge

이 프로젝트는 **F1TENTH Gym 환경을 ROS2 시뮬레이션으로 확장**하기 위한 **Docker 기반 ROS2 통신 브릿지**입니다.  
컨테이너 환경에서 시뮬레이션을 실행하고, RViz2를 통한 시각화 및 키보드 원격조종이 가능합니다.

---

## 📘 개요

이 환경을 통해 다음을 수행할 수 있습니다:
- F1TENTH Gym을 **Docker 컨테이너 내에서 실행**
- **RViz2**로 시각화
- **Keyboard Teleoperation**을 통한 주행 제어
- **PID, AEB 등 기능 확장 가능**
- **ROS2 Topic 통신**을 이용한 로컬-컨테이너 간 연동

---

## 🧩 PID 설정 참고 링크

PID 제어기 튜닝 및 시뮬레이션 참고:  
🔗 [PID Simulator – tech-uofm.info/pid/pid.html](https://tech-uofm.info/pid/pid.html)

---

## 🐳 Docker 환경 설정

> ⚠️ Docker 및 Docker Compose가 설치되어 있어야 합니다.

### 1) Docker 이미지 빌드
'''bash
cd f1tenth_gym_ros/  
docker build -t f1tenth_gym_ros -f Dockerfile .
'''

### 2) Docker 컨테이너 실행
docker-compose up

✅ 설명  
docker-compose up 명령어는 docker-compose.yml 파일의 설정에 따라 컨테이너를 실행하고, 환경 전체를 자동 구성합니다.  
실행 중인 컨테이너 확인:  
docker ps

### 3) Docker 접속 및 실행
브라우저에서 다음 주소 접속:  
👉 http://localhost:8080/vnc.html  

이후 터미널에서 다음 명령어 실행:  
docker exec -it f1tenth_gym_ros_sim_1 /bin/bash  

컨테이너 내부에서 아래 명령어 실행:  
source /opt/ros/foxy/setup.bash  
source install/local_setup.bash  
ros2 launch f1tenth_gym_ros gym_bridge_launch.py  

✅ 이제 F1TENTH Gym 시뮬레이션이 실행됩니다.

---

## 🕹️ 키보드 텔레오퍼레이션 (Keyboard Teleoperation)

로컬 환경에서 아래 명령어를 실행합니다:  
cd f1tenth_gym_ros/launch  
rviz2 -d gym_bridge.rviz  

이제 키보드를 이용하여 차량을 직접 제어할 수 있습니다.

---

## 🏁 상대 차량(Opponent)과 함께 실행하기

상대 차량을 추가하려면 설정 파일을 수정합니다:  
# f1tenth_gym_ros/config/sim.yaml  
num_agent: 2  

그 다음, 컨테이너 내부에서 아래 명령어 실행:  
colcon build  
source /opt/ros/foxy/setup.bash  
source install/local_setup.bash  
ros2 launch f1tenth_gym_ros gym_bridge_launch.py  

RViz에서 RobotModel을 추가하고, description을 다음과 같이 설정하면  
상대 차량 시각화가 가능합니다:  
/opp_robot_description

---

## ⚙️ 코드 수정 및 빌드 반영

Python 파일을 수정하거나 새로운 패키지를 추가할 경우 항상 재빌드해야 합니다.

### 코드 변경 반영 명령어
colcon build  
source /opt/ros/foxy/setup.bash  
source install/local_setup.bash  

⚠️ 매번 colcon build 이후 반드시 두 개의 source 명령어를 실행해야 변경 사항이 적용됩니다.

---

## 🚨 예시: AEB (자동 긴급 제동) 기능 추가

### Step 1. 새 패키지 생성
cd f1tenth_labs_ws  
ros2 pkg create --build-type ament_python --node-name safety_node lab2  
colcon build  
source install/local_setup.bash  
ros2 run lab2 safety_node  

### Step 2. Publisher 노드 작성
Python 코드 수정 후 아래 명령어 실행:  
colcon build  
ros2 run lab2 safety_node  

활성화된 노드 확인:  
ros2 node list  

ROS 토픽 관련 명령어:  
ros2 topic list  
ros2 topic info /brake_bool --verbose  
ros2 topic echo /brake_bool  

### Step 3. Subscriber 추가
safety_node 파일에서 ego 차량의 /scan 토픽(LiDAR 데이터)을 구독하도록 작성합니다.  
이를 통해 주변 장애물 감지 시 /brake_bool을 발행하여 제동 기능을 수행할 수 있습니다.

---

## ✅ 명령어 요약표

| 작업 | 명령어 / 설명 |
|------|----------------|
| Docker 이미지 빌드 | docker build -t f1tenth_gym_ros -f Dockerfile . |
| 컨테이너 실행 | docker-compose up |
| 컨테이너 접속 | docker exec -it f1tenth_gym_ros_sim_1 /bin/bash |
| 환경 설정 | source /opt/ros/foxy/setup.bash && source install/local_setup.bash |
| 시뮬레이션 실행 | ros2 launch f1tenth_gym_ros gym_bridge_launch.py |
| 텔레오퍼레이션 실행 | rviz2 -d gym_bridge.rviz |
| 코드 수정 후 빌드 | colcon build |

---

## 🧰 문제 해결 (Troubleshooting)

1. **RViz에서 로봇 모델이 보이지 않을 때**  
   - RViz에서 RobotModel을 추가했는지 확인  
   - Description이 /ego_robot_description 또는 /opp_robot_description으로 설정되어 있는지 확인  

2. **Topic이 구독되지 않을 때**  
   - ros2 topic list로 활성화된 토픽 목록을 확인  
   - source 명령어를 빼먹지 않았는지 점검  

3. **colcon build 실패 시**  
   - Python 코드 문법 오류 또는 __init__.py 누락 여부 확인  
   - 종속 패키지가 설치되어 있는지 확인:  
     rosdep install --from-paths src --ignore-src -r -y  

4. **Docker 컨테이너 접속 오류 시**  
   - 실행 중인 컨테이너 확인:  
     docker ps  
   - 컨테이너 이름이 f1tenth_gym_ros_sim_1이 맞는지 확인 후 다시 접속 시도  

---

## 📦 참고

- ROS2 버전: **Foxy**  
- 시뮬레이터: **F1TENTH Gym**  
- Docker를 이용하여 환경 의존성 최소화  
- 기능 추가 시 반드시 colcon build + source 명령어 실행 필수  

---
