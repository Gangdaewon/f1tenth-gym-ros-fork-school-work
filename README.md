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
🔗 [PID Simulator](https://tech-uofm.info/pid/pid.html)

---

## 🐳 Docker 환경 설정

> ⚠️ **사전 요구사항**: Docker 및 Docker Compose가 설치되어 있어야 합니다.

### 1️⃣ Docker 이미지 빌드

```bash
cd f1tenth_gym_ros/
docker build -t f1tenth_gym_ros -f Dockerfile .
```

### 2️⃣ Docker 컨테이너 실행

```bash
docker-compose up
```

**✅ 실행 결과**
- `docker-compose up` 명령어는 `docker-compose.yml` 파일의 설정에 따라 컨테이너를 실행하고, 환경 전체를 자동 구성합니다.
- 실행 중인 컨테이너 확인:

```bash
docker ps
```

### 3️⃣ Docker 접속 및 실행

**웹 브라우저로 접속:**
👉 http://localhost:8080/vnc.html

**터미널에서 Docker 컨테이너 접속:**
```bash
docker exec -it f1tenth_gym_ros_sim_1 /bin/bash
```

**컨테이너 내부에서 시뮬레이션 실행:**
```bash
# ROS2 환경 설정
source /opt/ros/foxy/setup.bash
source install/local_setup.bash

# F1TENTH Gym 시뮬레이션 실행
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

✅ **이제 F1TENTH Gym 시뮬레이션이 실행됩니다!**

---

## 🕹️ 키보드 텔레오퍼레이션 (Keyboard Teleoperation)

**로컬 환경에서 RViz2 실행:**
```bash
cd f1tenth_gym_ros/launch
rviz2 -d gym_bridge.rviz
```

이제 키보드를 이용하여 차량을 직접 제어할 수 있습니다! 🎮

---

## 🏁 상대 차량(Opponent)과 함께 실행하기

### 1️⃣ 설정 파일 수정
`f1tenth_gym_ros/config/sim.yaml` 파일에서 상대 차량 수를 설정합니다:

```yaml
# opponent parameters
num_agent: 2
```

### 2️⃣ 컨테이너에서 재빌드 및 실행
```bash
# 패키지 재빌드
colcon build

# 환경 설정 (필수!)
source /opt/ros/foxy/setup.bash
source install/local_setup.bash

# 시뮬레이션 실행
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### 3️⃣ RViz에서 상대 차량 시각화
- RViz에서 **RobotModel** 추가
- **Description**을 `/opp_robot_description`으로 설정
- 상대 차량 시각화 완료! 🚗💨

---

## ⚙️ 기능 추가 및 개발 가이드

### 📌 중요 사항
- Python 파일을 수정하거나 새로운 패키지를 추가할 경우 **반드시 재빌드**가 필요합니다.
- 매번 `colcon build` 이후 **반드시 source 명령어를 실행**해야 변경 사항이 적용됩니다.

### 🔄 코드 변경 반영 워크플로우
```bash
# 1. 패키지 빌드
colcon build

# 2. 환경 설정 (필수!)
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
```

---

## 🚨 예시: AEB (자동 긴급 제동) 기능 추가

### Step 1️⃣: 새 패키지 생성
```bash
cd f1tenth_labs_ws
ros2 pkg create --build-type ament_python --node-name safety_node lab2
colcon build
source install/local_setup.bash
ros2 run lab2 safety_node
```

### Step 2️⃣: Publisher 노드 작성
Python 코드 수정 후 아래 명령어 실행:
```bash
# 패키지 빌드
colcon build

# 노드 실행
ros2 run lab2 safety_node
```

**📊 노드 및 토픽 확인:**
```bash
# 활성화된 노드 목록 확인
ros2 node list

# ROS 토픽 관련 명령어
ros2 topic list
ros2 topic info /brake_bool --verbose
ros2 topic echo /brake_bool
```

### Step 3️⃣: Subscriber 추가
`safety_node` 파일에서 ego 차량의 `/scan` 토픽(LiDAR 데이터)을 구독하도록 작성합니다.  
이를 통해 주변 장애물 감지 시 `/brake_bool`을 발행하여 제동 기능을 수행할 수 있습니다.

---

## ✅ 명령어 요약표

| 작업 | 명령어 |
|------|--------|
| **Docker 이미지 빌드** | `docker build -t f1tenth_gym_ros -f Dockerfile .` |
| **컨테이너 실행** | `docker-compose up` |
| **컨테이너 접속** | `docker exec -it f1tenth_gym_ros_sim_1 /bin/bash` |
| **환경 설정** | `source /opt/ros/foxy/setup.bash && source install/local_setup.bash` |
| **시뮬레이션 실행** | `ros2 launch f1tenth_gym_ros gym_bridge_launch.py` |
| **텔레오퍼레이션** | `rviz2 -d gym_bridge.rviz` |
| **코드 빌드** | `colcon build` |

---

## 🧰 문제 해결 (Troubleshooting)

### 1️⃣ RViz에서 로봇 모델이 보이지 않을 때
- ✅ RViz에서 **RobotModel**을 추가했는지 확인
- ✅ **Description**이 `/ego_robot_description` 또는 `/opp_robot_description`으로 설정되어 있는지 확인

### 2️⃣ Topic이 구독되지 않을 때
- ✅ 다음 명령어로 활성화된 토픽 목록을 확인:
```bash
ros2 topic list
```
- ✅ **source 명령어**를 빼먹지 않았는지 점검

### 3️⃣ colcon build 실패 시
- ✅ Python 코드 문법 오류 또는 `__init__.py` 누락 여부 확인
- ✅ 종속 패키지 설치:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 4️⃣ Docker 컨테이너 접속 오류 시
- ✅ 실행 중인 컨테이너 확인:
```bash
docker ps
```
- ✅ 컨테이너 이름이 `f1tenth_gym_ros_sim_1`이 맞는지 확인 후 다시 접속 시도

---

## 📦 기술 스택

- **ROS2 버전**: Foxy Fitzroy
- **시뮬레이터**: F1TENTH Gym
- **컨테이너**: Docker & Docker Compose
- **시각화**: RViz2
- **언어**: Python

---

## 🎯 주요 특징

- 🐳 **Docker 기반 환경 격리**로 의존성 문제 최소화
- 🎮 **키보드 텔레오퍼레이션** 지원
- 🤖 **멀티 에이전트** 시뮬레이션 가능
- 📊 **ROS2 토픽 통신**을 통한 확장성
- 🔧 **모듈식 구조**로 기능 추가 용이

---

**Happy Racing! 🏎️💨**







