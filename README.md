# mirobot_transform_test

ROS 2 Humble 테스트 패키지.
좌표 변환 코드(aruco_tf) 없이 **mec_wheel의 `stopped` → ArUco 샘플링 → MoveIt 전달 준비**
파이프라인 전체가 정상 동작하는지 확인하기 위한 테스트 모드입니다.

---

## 테스트 모드 파라미터 3가지

| 파라미터 | 기본값 | 오늘 값 | 의미 |
|----------|--------|---------|------|
| `use_tf_transform` | `true` | **`false`** | TF 변환 없이 카메라 좌표 그대로 통과 |
| `dry_run_only` | `false` | **`true`** | MoveIt 호출 없이 로그만 출력 (move_group 불필요) |
| `accept_any_frame` | `false` | **`true`** | 카메라 프레임 이름으로 들어와도 frame_id 검증 통과 |

---

## 커맨드 입력 순서

### 사전 준비 — 빌드 (최초 1회)

```bash
cd ~/ros2_ws
colcon build --packages-select mirobot_moveit_tracker mec_wheel
source install/setup.bash
```

---

### 터미널 1 — D405 카메라 노드 (조원 담당)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

---

### 터미널 2 — ArUco 마커 인식 노드 (조원 담당)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ros2_aruco aruco_recognition.launch.py
```

카메라에 ArUco 마커를 갖다 대면 `/aruco_poses` 토픽이 발행됩니다.
확인 방법:

```bash
ros2 topic echo /aruco_poses
```

---

### 터미널 3 — mec_wheel 노드

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mec_wheel mec_wheel.launch.py
```

`blind_orbit` → `align_orbit` → `approach` → `stop` 순서로 상태가 전환됩니다.
`stop` 상태가 되면 `/wheel_status` 토픽으로 `"stopped"` 문자열이 발행됩니다.

---

### 터미널 4 — mirobot_transform_test (테스트 모드)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mirobot_moveit_tracker mirobot_moveit_tracker.launch.py \
    use_tf_transform:=false \
    dry_run_only:=true \
    accept_any_frame:=true
```

---

### (선택) 토픽 상태 모니터링

```bash
# /wheel_status 확인
ros2 topic echo /wheel_status

# /aruco_poses 확인
ros2 topic echo /aruco_poses

# /mirobot_goal_pose 확인 (stopped 이후 발행됨)
ros2 topic echo /mirobot_goal_pose
```

---

## 정상 동작 시 기대 로그

### 터미널 3 — mec_wheel

```
[mec_wheel_node] blind_orbit
[mec_wheel_node] blind_orbit
[mec_wheel_node] align_orbit | w: 0.87
[mec_wheel_node] align_orbit | w: 0.12
[mec_wheel_node] approach
[mec_wheel_node] approach
[mec_wheel_node] stop
[mec_wheel_node] stop
```

`stop` 로그가 뜨는 순간 `/wheel_status: "stopped"` 가 발행됩니다.

---

### 터미널 4 — mirobot_transform_test

**노드 기동 직후 (1회 출력):**

```
[moveit_goal_node] MoveItGoalNode parameter
  group        : mirobot_group
  base_link    : base_link
  end_effector : link6
  joint_names  : ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  cartesian    : False
  execute      : True
  dry_run_only : True
  accept_frame : True
```

**mec_wheel에서 `stopped` 발행 직후:**

```
[wheel_stop_to_goal_node] Wheel stopped. Waiting 0.200 sec before collecting 5 samples.
```

**0.2초 후 샘플 수집 시작:**

```
[wheel_stop_to_goal_node] Started ArUco pose sampling.
```

**5개 샘플 수집 완료 및 평균 좌표 발행:**

```
[wheel_stop_to_goal_node] Averaged pose published (in camera_color_optical_frame): x=0.XXXX y=0.XXXX z=0.XXXX
```

**moveit_goal_node가 좌표 수신:**

```
[moveit_goal_node] Received MoveIt goal request: frame=camera_color_optical_frame x=0.XXXX y=0.XXXX z=0.XXXX
```

**좌표 검증 통과:**

```
[moveit_goal_node] Goal validated. Ready to send to MoveIt: pos=[0.XXXX, 0.XXXX, 0.XXXX] quat=[0.XXXX, 0.XXXX, 0.XXXX, 0.XXXX]
```

**dry-run 완료 (MoveIt 미호출):**

```
[moveit_goal_node] Dry-run only: tracker is ready. Goal would be sent to MoveIt here.
```

---

## 오늘 테스트 성공 조건

위 로그가 순서대로 모두 출력되면 다음이 확인된 것입니다.

- `mec_wheel`의 `stopped` 토픽을 `mirobot_tracker`가 정상 수신 ✅
- `stopped` 이후 ArUco 좌표 5개 샘플링 및 평균 계산 정상 동작 ✅
- 계산된 좌표가 `moveit_goal_node`까지 정상 전달 ✅
- MoveIt으로 보낼 준비 완료 ✅

---

## 비정상 로그와 원인

### `"No ArUco pose received yet."` 반복

```
[wheel_stop_to_goal_node] No ArUco pose received yet.
```

`stopped`가 왔을 때 아직 `/aruco_poses`가 수신된 적 없는 상태입니다.
터미널 2(ArUco 인식 노드)가 실행 중인지, 마커가 카메라에 보이는지 확인합니다.

```bash
ros2 topic echo /aruco_poses
```

---

### `stopped` 이후 아무 반응 없음

`/wheel_status` 토픽이 `mirobot_tracker`에 도달하지 않은 상태입니다.
토픽 발행 여부를 확인합니다.

```bash
ros2 topic echo /wheel_status
```

---

### `"Failed to send goal to MoveIt: Goal pose has invalid zero quaternion."`

```
[moveit_goal_node] Failed to send goal to MoveIt: Goal pose has invalid zero quaternion.
```

ArUco 마커의 orientation이 모두 0으로 들어오는 상태입니다.
`ros2_aruco`가 quaternion을 올바르게 계산하는지 확인합니다.

```bash
ros2 topic echo /aruco_poses
# orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0} 이면 문제
```

---

## 실제 운용 전환 시

좌표 변환(aruco_tf) 코드가 준비되면 아래 명령으로 전환합니다.

```bash
ros2 launch mirobot_moveit_tracker mirobot_moveit_tracker.launch.py \
    use_tf_transform:=true \
    dry_run_only:=false \
    accept_any_frame:=false
```
