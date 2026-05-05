# mirobot_moveit_tracker

ROS 2 Humble 패키지. 매카넘 휠 동체가 정지(`stopped`)한 시점에 D405 카메라로 인식한 ArUco 마커 좌표를 TF 변환하여 MoveIt2로 전달하고, WLKATA Mirobot 팔의 trajectory를 생성·실행합니다.

---

## 프로젝트 개요

매카넘 휠 위에 WLKATA Mirobot과 Intel D405 카메라를 탑재한 시스템입니다.

1. 동체가 ArUco 마커가 부착된 dummy로부터 1m 이상 떨어진 곳에서 출발합니다.
2. dummy와의 거리가 50cm가 되면 정지하고, 반시계 방향으로 dummy 주위를 회전하며 ArUco 마커를 탐색합니다.
3. 마커가 감지되면 마커와 일직선으로 정렬(align)한 뒤 30cm 거리까지 직진합니다.
4. **동체가 정지한 시점부터 이 패키지가 동작합니다.** D405로부터 마커 좌표를 수집·변환하여 MoveIt2로 전달하고, Mirobot 팔을 뻗어 마커로부터 1cm 앞의 물체를 파지합니다.

---

## 시스템 구성 및 토픽 흐름

```
[D405 카메라]
      │  /camera/image_raw
      ▼
[ros2_aruco]  ──────────────────────────────────────────┐
      │  /aruco_poses (PoseArray, 카메라 프레임 기준)     │
      ▼                                                  │
[mec_wheel_node]                                         │
      │  /wheel_status ("stopped")                       │
      ▼                                                  ▼
[wheel_stop_to_goal_node] ◄─────────────── /aruco_poses
      │  TF 변환 (카메라 프레임 → base_link)
      │  5개 샘플 수집 → 평균 계산 → offset 적용
      │  /mirobot_goal_pose (PoseStamped, base_link 기준)
      ▼
[moveit_goal_node]
      │  frame_id / quaternion 검증
      │  MoveIt2.move_to_pose() 호출
      ▼
[MoveIt2 / move_group]
      │  IK 계산 → 충돌 검사 → trajectory 생성
      ▼
[WLKATA Mirobot 실행]
```

---

## 패키지 구조

```
mirobot_moveit_tracker/
├── mirobot_moveit_tracker/
│   ├── __init__.py
│   ├── wheel_stop_to_goal_node.py   # /wheel_status 수신 → TF 변환 → goal 발행
│   └── moveit_goal_node.py          # goal 수신 → MoveIt2 trajectory 실행
├── launch/
│   └── mirobot_moveit_tracker.launch.py
├── resource/
│   └── mirobot_moveit_tracker
├── package.xml
├── setup.py
└── setup.cfg
```

---

## 의존성

### apt 패키지

```bash
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
```

### pymoveit2

MoveIt2의 Python 클라이언트 라이브러리입니다. 설치 여부를 먼저 확인합니다.

```bash
python3 -c "from pymoveit2 import MoveIt2; print('OK')"
```

설치되어 있지 않다면 소스 빌드합니다.

```bash
cd ~/ros2_ws/src
git clone https://github.com/AndrejOrsula/pymoveit2
cd ~/ros2_ws
colcon build --packages-select pymoveit2
source install/setup.bash
```

---

## 빌드

```bash
cd ~/ros2_ws

# 패키지를 src 디렉토리에 복사한 후
colcon build --packages-select mirobot_moveit_tracker
source install/setup.bash
```

---

## 실행 방법

전체 시스템은 터미널 4개에서 순서대로 실행합니다.  
TF 트리(`camera_color_optical_frame → base_link`) 브로드캐스트는 조원의 전체 시스템 launch 파일이 담당합니다.

```bash
# 터미널 1 — D405 카메라 노드 (조원 담당)
ros2 launch realsense2_camera rs_launch.py

# 터미널 2 — ArUco 마커 인식 노드 (조원 담당)
ros2 launch ros2_aruco aruco_recognition.launch.py

# 터미널 3 — 매카넘 휠 노드
ros2 launch mec_wheel mec_wheel.launch.py

# 터미널 4 — mirobot_tracker (이 패키지)
ros2 launch mirobot_moveit_tracker mirobot_moveit_tracker.launch.py
```

### launch 파라미터 오버라이드 예시

Mirobot SRDF의 joint 이름이 다를 경우, 또는 offset을 조정할 경우 다음과 같이 오버라이드합니다.

```bash
ros2 launch mirobot_moveit_tracker mirobot_moveit_tracker.launch.py \
    joint_names:="[j1, j2, j3, j4, j5, j6]" \
    offset_z:="-0.01"
```

---

## launch 파라미터 설명

### wheel_stop_to_goal_node

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `pose_topic` | `/aruco_poses` | ros2_aruco가 발행하는 PoseArray 토픽 |
| `wheel_status_topic` | `/wheel_status` | mec_wheel이 발행하는 상태 토픽 |
| `goal_topic` | `/mirobot_goal_pose` | 두 노드 사이 중간 토픽 |
| `sample_delay_sec` | `0.2` | stopped 감지 후 수집 시작까지 대기 시간(초) |
| `sample_count` | `5` | 평균 계산에 사용할 샘플 수 |
| `offset_x` | `0.0` | base_link 기준 X 오프셋(m) |
| `offset_y` | `0.0` | base_link 기준 Y 오프셋(m) |
| `offset_z` | `0.0` | base_link 기준 Z 오프셋(m) |
| `goal_frame` | `base_link` | TF 변환 목적지 프레임 (`base_link_name`과 동일해야 함) |
| `use_marker_orientation` | `true` | true: 변환된 마커 orientation 사용 / false: goal_q* 고정값 사용 |
| `tf_timeout_sec` | `0.5` | TF 조회 최대 대기 시간(초) |

### moveit_goal_node

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `group_name` | `mirobot_group` | MoveIt planning group 이름 (SRDF와 일치해야 함) |
| `base_link_name` | `base_link` | MoveIt base frame 이름 (goal_frame과 동일해야 함) |
| `end_effector_name` | `link6` | End-effector link 이름 |
| `joint_names` | `[joint1~joint6]` | Planning group의 joint 이름 목록 (SRDF와 일치해야 함) |
| `cartesian` | `false` | true: Cartesian path 계획 / false: joint-space 계획 |
| `execute` | `true` | true: 계획 후 즉시 실행 / false: 계획만 수행 |
| `ignore_same_goal` | `true` | 직전 goal과 동일하면 무시 |

---

## 정상 동작 로그

D405 카메라에 ArUco 마커를 갖다 대고 mec_wheel에서 `stopped`가 발행되면, `mirobot_tracker` 터미널에 다음 순서로 로그가 출력됩니다.

```
# 노드 초기화 시
[moveit_goal_node] MoveItGoalNode parameter
  group        : mirobot_group
  base_link    : base_link
  end_effector : link6
  joint_names  : ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  cartesian    : False
  execute      : True

# mec_wheel이 stopped 발행 직후
[wheel_stop_to_goal_node] Wheel stopped. Waiting 0.200 sec before collecting 5 samples.

# 0.2초 후 샘플 수집 시작
[wheel_stop_to_goal_node] Started ArUco pose sampling.

# 5개 샘플 수집 완료 및 goal 발행
[wheel_stop_to_goal_node] Averaged pose published (in base_link): x=0.XXXX y=0.XXXX z=0.XXXX

# moveit_goal_node가 goal 수신
[moveit_goal_node] Received MoveIt goal request: frame=base_link x=0.XXXX y=0.XXXX z=0.XXXX

# 검증 통과 후 MoveIt 전송
[moveit_goal_node] Goal validated. Ready to send to MoveIt: pos=[...] quat=[...]
[moveit_goal_node] Sending joint-space goal to MoveIt.

# trajectory 실행 완료
[moveit_goal_node] MoveIt goal execution completed.
```

---

## 문제 해결

### TF 변환 실패 (`LookupException`)

```
[wheel_stop_to_goal_node] [TF] LookupException  camera_color_optical_frame -> base_link : ...
[wheel_stop_to_goal_node] TF transform failed.
```

TF 트리가 연결되지 않은 상태입니다. 조원의 시스템 launch가 실행 중인지 확인합니다.

```bash
# TF 트리 시각화
ros2 run tf2_tools view_frames

# 두 프레임 간 변환 확인
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

### joint_names 불일치

MoveIt이 IK를 찾지 못하는 경우 SRDF의 실제 joint 이름을 확인합니다.

```bash
ros2 param get /move_group robot_description_semantic
```

확인한 이름으로 launch 파라미터를 오버라이드합니다.

```bash
ros2 launch mirobot_moveit_tracker mirobot_moveit_tracker.launch.py \
    joint_names:="[실제joint1, 실제joint2, ...]"
```

### pymoveit2 import 오류

```
ModuleNotFoundError: No module named 'pymoveit2'
```

pymoveit2가 설치되지 않은 것입니다. 의존성 섹션의 소스 빌드 방법을 따릅니다.

---

## 노드별 역할 요약

**wheel_stop_to_goal_node**
- `/wheel_status`에서 `"stopped"` 수신 시 동작 시작
- `/aruco_poses`에서 샘플 5개 수집
- 각 샘플을 `tf_buffer.transform()`으로 `base_link` 기준으로 변환
- 5개 샘플의 위치 평균 계산 후 offset 적용
- `/mirobot_goal_pose`로 `PoseStamped` 발행

**moveit_goal_node**
- `/mirobot_goal_pose` 수신
- `frame_id`, quaternion 유효성 검증
- `MultiThreadedExecutor` 환경에서 `pymoveit2.MoveIt2.move_to_pose()` 호출
- `wait_until_executed()`로 trajectory 실행 완료 대기
