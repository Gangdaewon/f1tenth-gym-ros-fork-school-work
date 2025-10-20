# 🏎️ F1TENTH Gym ROS2 통신 브리지 (Communication Bridge)

이 프로젝트는 [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym) 환경을 **ROS2 시뮬레이션 환경으로 확장**하기 위한  
**컨테이너 기반 ROS2 통신 브리지(Containerized ROS2 Communication Bridge)** 입니다.

Docker를 이용해 시뮬레이션 환경을 자동으로 구성하며,  
**키보드 텔레오퍼레이션**, **상대 차량(opponent) 시뮬레이션**, **기능 확장(AEB, Planning 등)** 이 가능합니다.

---

## 📦 1. 사전 준비 (Prerequisites)

ROS2 환경을 구성하기 전에 **Docker**와 **Docker Compose**를 설치해야 합니다.

```bash
# Docker 및 Docker Compose 설치
sudo apt update
sudo apt install docker.io docker-compose -y
🧱 2. Docker 이미지 빌드
bash
코드 복사
cd f1tenth_gym_ros/
docker build -t f1tenth_gym_ros -f Dockerfile .
🚀 3. 시뮬레이션 실행
bash
코드 복사
docker-compose up
✅ 설명

docker-compose.yml 파일에 정의된 모든 컨테이너를 자동으로 실행 및 연결합니다.

전체 시뮬레이션 환경이 자동으로 구성됩니다.

실행 중인 컨테이너 확인:

bash
코드 복사
docker ps
🌐 4. 웹 인터페이스 접속
컨테이너 실행 후, 브라우저에서 아래 주소로 접속합니다:

bash
코드 복사
http://localhost:8080/vnc.html
🐚 5. 컨테이너 내부 접속
bash
코드 복사
docker exec -it f1tenth_gym_ros_sim_1 /bin/bash
컨테이너 내부 프롬프트(root@xxxx:/sim_ws#)에서 ROS2를 설정하고 시뮬레이션을 실행합니다:

bash
코드 복사
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
➡️ F1TENTH Gym 시뮬레이션이 ROS2 환경에서 구동됩니다.

⌨️ 6. 키보드 텔레오퍼레이션 (Keyboard Teleoperation)
호스트(내 컴퓨터)에서 아래 명령어를 실행합니다:

bash
코드 복사
cd f1tenth_gym_ros/launch
rviz2 -d gym_bridge.rviz
✅ RViz에서 키보드로 차량 조종이 가능합니다.

🏁 7. 상대 차량(Opponent) 추가 실행
f1tenth_gym_ros/config/sim.yaml 파일을 열어 아래 항목을 수정합니다:

yaml
코드 복사
# opponent parameters
num_agent: 2
컨테이너 내부에서 다시 빌드 및 실행합니다:

bash
코드 복사
colcon build
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
RViz에서 RobotModel을 추가하고, 토픽을 /opp_robot_description으로 설정하면
상대 차량이 시각화됩니다.

🔄 8. 기능 확장 (예: AEB 추가)
새로운 ROS2 패키지를 생성하여 기능을 확장할 수 있습니다.

🧩 Step 1. 패키지 생성
bash
코드 복사
cd f1tenth_labs_ws
ros2 pkg create --build-type ament_python --node-name safety_node lab2
colcon build
source install/local_setup.bash
ros2 run lab2 safety_node
📝 Step 2. Publisher 노드 작성
Python 노드를 수정한 뒤 다음 명령으로 실행합니다:

bash
코드 복사
colcon build
ros2 run lab2 safety_node
활성화된 노드 목록을 확인:

bash
코드 복사
ros2 node list
토픽 관련 정보 확인 명령어:

bash
코드 복사
ros2 topic list
ros2 topic info /brake_bool --verbose
ros2 topic echo /brake_bool
📡 Step 3. Subscriber 추가
ego_vehicle의 /scan 토픽을 **구독(subscribe)**하도록 Python 코드에 기능을 추가합니다.
수정 후 빌드 및 재실행합니다:

bash
코드 복사
colcon build
source install/local_setup.bash
ros2 run lab2 safety_node
⚙️ 9. 빌드 및 변경사항 적용
Python 파일 또는 launch 파일을 수정한 후에는 반드시 아래 명령을 다시 실행해야 합니다.

bash
코드 복사
colcon build
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
📘 10. PID 제어 시뮬레이터 (참고용)
PID 제어 파라미터 튜닝을 위한 온라인 시뮬레이터:
👉 https://tech-uofm.info/pid/pid.html

🗂️ 11. 디렉토리 구조 (Directory Structure)
아래는 기본적인 프로젝트 디렉토리 구성 예시입니다.

csharp
코드 복사
f1tenth_gym_ros/
├── config/
│   ├── sim.yaml                  # 시뮬레이션 및 opponent 설정
│   └── params.yaml               # PID 또는 환경 파라미터 (선택사항)
│
├── launch/
│   ├── gym_bridge_launch.py      # ROS2 브리지 실행 파일
│   └── gym_bridge.rviz           # RViz 텔레오퍼레이션 설정 파일
│
├── src/
│   ├── f1tenth_gym_ros/          # ROS2 브리지 및 노드 코드
│   ├── lab2/                     # (예시) AEB와 같은 추가 패키지
│   └── ...
│
├── Dockerfile                    # 컨테이너 환경 정의 파일
├── docker-compose.yml            # 멀티 컨테이너 실행 설정
├── install/                      # colcon 빌드 결과물
├── build/                        # colcon 빌드 캐시
└── README.md
✅ 12. 요약 (Summary)
단계	명령어 / 파일	설명
1	docker build -t f1tenth_gym_ros -f Dockerfile .	Docker 이미지 빌드
2	docker-compose up	시뮬레이션 실행
3	docker exec -it f1tenth_gym_ros_sim_1 /bin/bash	컨테이너 내부 진입
4	ros2 launch f1tenth_gym_ros gym_bridge_launch.py	ROS2 브리지 실행
5	rviz2 -d gym_bridge.rviz	키보드 조종 (텔레오퍼레이션)
6	colcon build && source install/local_setup.bash	코드 변경사항 반영

🧠 13. 참고 사항 (Notes)
모든 빌드 후에는 반드시 다음 명령어를 실행해야 합니다:

bash
코드 복사
source /opt/ros/foxy/setup.bash && source install/local_setup.bash
Python 파일 수정 후 반드시 colcon build를 다시 수행해야 합니다.

rviz2가 실행되지 않는다면 X11 디스플레이 포워딩 설정을 확인하세요.

docker ps로 실행 중인 컨테이너 이름을 확인 후 docker exec 명령을 실행하세요.