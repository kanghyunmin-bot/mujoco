# UUV MuJoCo v2.2 Code Guide (SITL/QGC 전용)

이 문서는 `run_urdf_full.py`의 현재 운영 경로를 SITL/QGC 기준으로 정리합니다.

## 1) `run_urdf_full.py`

### 1.1 역할

- SITL(MAVLink + JSON) 입력을 받아 MuJoCo 쓰러스터 제어로 변환
- IMU/DVL/카메라를 ROS2 토픽으로 발행(옵션)
- `thr_target -> data.ctrl` 물리 반영

### 1.2 핵심 실행 흐름

1. CLI 파싱 (`--sitl`, `--sitl-servo-*`, `--ros2*`)
2. MuJoCo 모델/액추에이터 로드
3. SITL 서보 수신(MAVLink `SERVO_OUTPUT_RAW` 또는 JSON)
4. 쓰러스터 명령 생성(direct mapping 또는 mixer inverse)
5. `update_thruster_forces()`로 `data.ctrl` 적용
6. `apply_underwater_wrench()`로 부력/감쇠 적용
7. `ros_bridge.publish()`로 센서/영상 발행

### 1.3 핵심 함수

| Function | Principle | 목적 |
|---|---|---|
| `mix_horizontal_thrusters` | 의사역행렬 기반 수평 4추진기 분배 | 전진/스웨이/요 동시 만족 |
| `build_horizontal_allocator` | site 위치 + gear 기반 alloc 행렬 구성 | 지오메트리 일관성 보장 |
| `build_vertical_allocator` | 수직 쓰러스터로 roll/pitch 보정 분배 | 자세 보정 토크 분산 |
| `update_thruster_forces` | `thr_target`을 실제 힘으로 변환 후 clamp | actuator 안정 적용 |
| `apply_underwater_wrench` | 부력 + 선형/각속 감쇠 | 과도 진동 완화 |

### 1.4 현재 제거된 기능

- 로컬 키보드/조이스틱 수동 입력
- 검증 모드(`--validate*`)
- 캘리브레이션 모드(`--calibrate*`)

## 2) `ros2_bridge.py`

### 2.1 역할

- `/cmd_vel` 및 `/mavros/rc/override` 입력 처리(옵션)
- `/imu/data`, `/dvl/*`, `/depth`, `/mujoco/ground_truth/pose` 발행
- `--ros2-images` 사용 시 `/stereo/*` 발행

### 2.2 주의사항

- SITL 모드에서 RC override 충돌 방지를 위해 기본적으로 `/mavros/rc/override`를 제한할 수 있음
- 센서 발행률은 `--ros2-sensor-hz`, 영상은 `--ros2-image-hz`로 분리

## 3) `urdf_full_scene.xml`

### 3.1 역할

- 쓰러스터 site 위치
- actuator gear 방향 벡터
- IMU/DVL/카메라 기준 site

### 3.2 튜닝 포인트

| Element | 조정 항목 |
|---|---|
| `<motor name="yaw_*" gear="...">` | 수평 추진 방향/요 토크 응답 |
| `<motor name="ver_*" gear="...">` | 수직 추진/상하 응답 |
| `<site name="thr_*">` | 추력 작용점(레버암) |
| `<site name="imu_site">`, `<site name="dvl_site">` | 센서 기준 프레임 |

## 4) JSON 설정 파일

### `sim_profiles.json`

- `sim_real`, `sim_fast` 물리 파라미터 묶음
- 부력, 감쇠, 타임스텝, 쓰러스터 최대힘 포함

### `thruster_tune.json`

- actuator gear 보정 벡터
- allocator 계산의 기준 방향

### `thruster_params.json`

- per-thruster `gain_scale`
- 비대칭 응답 보정

## 5) 운영 체크리스트

1. ArduSub를 `--model JSON` + `--out ...:14660`으로 실행
2. MuJoCo를 `--sitl --sitl-servo-source mavlink`로 실행
3. QGC에서 arm/mode 전환 후 입력 시 `mujoco.log`의 servo 수신 로그 확인
4. 출력 방향 이상 시 `--sitl-servo-map`, `--sitl-servo-signs` 우선 조정
