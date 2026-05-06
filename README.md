# Mirobot 동작 파이프라인

ArUco 마커 인식 → 매카넘 휠 정지 → MoveIt trajectory 생성 → G-code 변환 → WLKATA Mirobot 실기 동작까지의 전체 과정을 설명합니다.

---

## 시스템 구성

```
[D405 카메라]
      │ /camera/image_raw
      ▼
[ros2_aruco]  ──── ArUco 마커 인식 ────►  /aruco_tf  (base_link 기준 좌표, PoseArray)
      │
      │ /aruco_poses
      ▼
[mec_wheel_node]
      │ /wheel_status ("stopped")
      ▼
[wheel_stop_to_goal_node] ◄──── /aruco_tf
      │ 5개 샘플 수집 → 평균 계산
      │ /mirobot_goal_pose (PoseStamped, base_link 기준)
      ▼
[moveit_goal_node]
      │ pymoveit2.move_to_pose()
      ▼
[MoveIt2 / move_group]
      │ IK 계산 → trajectory 생성
      │ FollowJointTrajectory 액션
      ▼
[MirobotGcodeDriver]
      │ radian → degree 변환
      │ M21 G90 G00 X.. Y.. Z.. A.. B.. C..
      ▼
[WLKATA Mirobot 실기 동작]
```

---

## 동작 조건 (모두 만족해야 함)

| 조건 | 확인 방법 |
|------|-----------|
| `/aruco_tf` 좌표가 base_link 기준 | `ros2 topic echo /aruco_tf` |
| quaternion이 모두 0이 아님 | `ros2 topic echo /aruco_tf` — orientation 값 확인 |
| `pymoveit2` 설치 | `python3 -c "from pymoveit2 import MoveIt2; print('OK')"` |
| `move_group` 실행 중 | `ros2 node list \| grep move_group` |
| goal pose가 Mirobot 작업공간 안 | MoveIt IK 계산 성공 여부로 확인 |

---

## 빌드 (최초 1회)

```bash
cd ~/ros2_ws

colcon build --packages-select \
    ros2_aruco \
    mec_wheel \
    mirobot_moveit_tracker \
    wlkata_mirobot_description \
    wlkata_mirobot_moveit_config \
    wlkata_arm_move

source install/setup.bash
```

### pymoveit2 설치 확인

```bash
python3 -c "from pymoveit2 import MoveIt2; print('OK')"
```

설치되어 있지 않으면:

```bash
cd ~/ros2_ws/src
git clone https://github.com/AndrejOrsula/pymoveit2
cd ~/ros2_ws
colcon build --packages-select pymoveit2
source install/setup.bash
```

---

## 노드 실행 순서

### 터미널 1 — D405 카메라 노드 (조원 담당)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

카메라가 정상 기동되면 `/camera/image_raw` 토픽이 발행됩니다.

---

### 터미널 2 — ArUco 인식 + aruco_tf 발행 (조원 담당)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ros2_aruco aruco_recognition.launch.py
```

D405 카메라에 ArUco 마커를 갖다 대면 `/aruco_tf` 토픽이 발행됩니다.

동작 확인:

```bash
ros2 topic echo /aruco_tf
```

정상 출력 예시:

```
header:
  frame_id: camera_color_optical_frame
poses:
- position:
    x: 0.041
    y: -0.018
    z: 0.182
  orientation:
    x: 0.983
    y: 0.037
    z: -0.072
    w: -0.164
```

orientation 이 모두 0.0 이면 마커 인식 오류입니다.

---

### 터미널 3 — mec_wheel

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mec_wheel mec_wheel.launch.py
```

---

### 터미널 4 — MoveIt + G-code 드라이버

`move_group`, `robot_state_publisher`, `MirobotGcodeDriver` 세 노드를 한 번에 실행합니다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch wlkata_mirobot_moveit_config real_hardware.launch.py \
    use_rviz:=false \
    use_joint_state_publisher:=true \
    serial_port:=/dev/ttyUSB0
```

시리얼 포트 확인:

```bash
ls /dev/ttyUSB*
```

---

### 터미널 5 — mirobot_tracker

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mirobot_moveit_tracker mirobot_moveit_tracker.launch.py
```

---

## 정상 동작 시 기대 로그

### 터미널 4 — real_hardware 기동 후

```
[robot_state_publisher] ...
[move_group] Ready to take commands for planning group mirobot_group.
[mirobot_gcode_driver] Serial opened: /dev/ttyUSB0 @ 115200 baud
[mirobot_gcode_driver] auto_home=False: homing skipped.
[mirobot_gcode_driver] Action server ready: /mirobot_group_controller/follow_joint_trajectory
```

### 터미널 5 — mirobot_tracker 기동 후

```
[moveit_goal_node] MoveItGoalNode parameter
  group        : mirobot_group
  base_link    : base_link
  end_effector : link6
  joint_names  : ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  cartesian    : False
  execute      : True
  dry_run_only : False
  accept_frame : False
```

### 터미널 3 — mec_wheel에서 stopped 발행

```
[mec_wheel_node] blind_orbit
[mec_wheel_node] align_orbit | w: 0.87
[mec_wheel_node] approach
[mec_wheel_node] stop
```

### 터미널 5 — stopped 수신 이후 순서대로 출력

```
[wheel_stop_to_goal_node] Wheel stopped. Waiting 0.200 sec before collecting 5 samples.
[wheel_stop_to_goal_node] Started ArUco pose sampling.
[wheel_stop_to_goal_node] Averaged pose published (in base_link): x=0.XXXX y=0.XXXX z=0.XXXX

[moveit_goal_node] Received MoveIt goal request: frame=base_link x=0.XXXX y=0.XXXX z=0.XXXX
[moveit_goal_node] Goal validated. Ready to send to MoveIt: pos=[...] quat=[...]
[moveit_goal_node] Sending joint-space goal to MoveIt.
[moveit_goal_node] MoveIt goal execution completed.
```

### 터미널 4 — trajectory 수신 및 G-code 전송

```
[mirobot_gcode_driver] Goal accepted: N waypoints | joints=['joint1', ...]
[mirobot_gcode_driver] Executing trajectory: N waypoints.
[mirobot_gcode_driver] [1/N] t=0.500s | M21 G90 G00 X1.72 Y0.00 Z0.00 A0.00 B0.00 C0.00
[mirobot_gcode_driver] [2/N] t=1.000s | M21 G90 G00 X...
...
[mirobot_gcode_driver] Trajectory complete: N waypoints sent.
___________________________________
```

이 로그가 순서대로 출력되면 Mirobot이 실제로 동작합니다.

---

## 문제 해결

### `No ArUco pose received yet.` 반복

```
[wheel_stop_to_goal_node] No ArUco pose received yet.
```

터미널 2가 실행 중인지, 마커가 카메라에 보이는지 확인합니다.

```bash
ros2 topic echo /aruco_tf
```

---

### `Goal pose has invalid zero quaternion.`

```
[moveit_goal_node] Failed to send goal to MoveIt: Goal pose has invalid zero quaternion.
```

`/aruco_tf`의 orientation이 모두 0.0입니다. 조원의 aruco_tf 변환 코드에서 quaternion이 올바르게 계산되는지 확인합니다.

---

### MoveIt IK 실패 (goal이 작업공간 밖)

```
[moveit_goal_node] Failed to send goal to MoveIt: ...
```

goal pose의 좌표가 Mirobot의 작업공간 밖에 있는 경우입니다. `/aruco_tf` 좌표값이 실제로 base_link 기준인지 조원과 확인하고, 필요 시 `offset_z` 파라미터로 조정합니다.

```bash
ros2 launch mirobot_moveit_tracker mirobot_moveit_tracker.launch.py \
    offset_z:=-0.05
```

---

### 시리얼 포트 오류

```
[mirobot_gcode_driver] Failed to open serial port /dev/ttyUSB0: ...
```

Mirobot USB 연결 상태와 포트 이름을 확인합니다.

```bash
ls /dev/ttyUSB*
# 포트가 다르면 serial_port 파라미터 변경
ros2 launch wlkata_mirobot_moveit_config real_hardware.launch.py \
    serial_port:=/dev/ttyUSB1 \
    use_rviz:=false \
    use_joint_state_publisher:=true
```

---

## 파라미터 요약

### mirobot_moveit_tracker.launch.py

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `pose_topic` | `/aruco_tf` | ArUco 좌표 토픽 |
| `use_tf_transform` | `false` | TF 변환 사용 여부 |
| `goal_frame` | `base_link` | 목표 좌표계 |
| `sample_count` | `5` | 평균 계산 샘플 수 |
| `offset_x/y/z` | `0.0` | base_link 기준 오프셋(m) |
| `dry_run_only` | `false` | true: MoveIt 미호출, 로그만 출력 |

### real_hardware.launch.py

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `serial_port` | `/dev/ttyUSB0` | Mirobot 시리얼 포트 |
| `use_rviz` | `true` | RViz 실행 여부 (라즈베리파이에서는 false 권장) |
| `use_joint_state_publisher` | `true` | joint state fallback 발행 |
| `auto_home` | `false` | 기동 시 자동 homing 여부 |
