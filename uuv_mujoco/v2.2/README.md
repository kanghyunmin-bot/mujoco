# UUV MuJoCo Simulator v2.2 (SITL/QGC 전용)

MuJoCo(`run_urdf_full.py`)는 ArduSub SITL과 MAVLink로 연동되고, QGroundControl에서 제어하는 경로만 유지합니다.

- 유지 경로: `SITL <-> MuJoCo <-> QGC`
- 제거 경로: 로컬 키보드/조이스틱 수동 제어, `--validate*`, `--calibrate*`

## 1) 가장 빠른 전체 실행

```bash
cd ~/antigravity
./scripts/stack_reset.sh --with-qgc-stop
./scripts/start_full_stack_direct.sh
```

로그는 `~/antigravity/log/full_stack_YYYYMMDD_HHMMSS/`에 저장됩니다.

## 2) 수동 3터미널 실행(권장 기준)

터미널 1: ArduSub SITL

```bash
cd ~/antigravity/kmu_hit25_ros2_ws/ardupilot
python3 Tools/autotest/sim_vehicle.py -L RATBeach --console --map \
  -v ArduSub -f vectored_6dof --model JSON \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14660
```

터미널 2: MuJoCo

```bash
cd ~/antigravity/mujoco/uuv_mujoco/v2.2
./launch_competition_sim.sh --sitl --images --force-clean \
  --sitl-servo-source mavlink \
  --sitl-mavlink-endpoint udpin:0.0.0.0:14660
```

터미널 3: QGroundControl

```bash
QGroundControl
```

## 3) 통신 파이프라인

- `14550/udp`: ArduSub -> QGC 텔레메트리
- `14551/udp`: ArduSub -> MAVROS(선택)
- `14660/udp`: ArduSub `SERVO_OUTPUT_RAW` -> MuJoCo(MAVLink 입력)
- `9002/9003/udp`: MuJoCo <-> ArduSub JSON 센서/서보 채널

SITL 모드에서는 내부 `imu_stabilize/depth_hold`가 자동 비활성화됩니다. 자세/심도 제어 권한은 ArduSub/QGC에 있습니다.

## 4) 기본 쓰러스터 매핑

`run_urdf_full.py` 기본값:

- `--sitl-servo-map yaw_rf,yaw_lf,yaw_rr,yaw_lr,ver_rf,ver_lf,ver_rr,ver_lr`
- `--sitl-servo-signs 1,1,1,1,-1,-1,-1,-1`

채널 해석은 `SERVO1..8` 순서로 적용됩니다.

## 5) 자주 쓰는 실행 옵션

직접 매핑(기본):

```bash
./launch_competition_sim.sh --sitl --images --force-clean \
  --sitl-servo-source mavlink \
  --sitl-direct-thrusters
```

역믹서 모드(필요 시만):

```bash
./launch_competition_sim.sh --sitl --images --force-clean \
  --sitl-servo-source mavlink \
  --no-sitl-direct-thrusters \
  --sitl-mixer-frame vectored_6dof
```

## 6) 트러블슈팅

QGC 연결 안 됨:

```bash
ss -lupn | rg '14550|14551|14660|9002|9003'
pkill -f "sim_vehicle.py|run_urdf_full.py|QGroundControl"
```

SITL이 바로 종료됨:

- `~/antigravity/log/full_stack_*/ardusub.log` 확인
- `--model JSON`, `--out ... 14660` 포함 여부 확인

제어가 중립만 들어옴:

- `mujoco.log`에 `servo stream is neutral` 반복 여부 확인
- SITL과 MuJoCo의 `--sitl-mavlink-endpoint` 포트 일치 확인

## 7) 문서

- 코드 구조: `docs/CODE_GUIDE.md`
- 파일 구조: `docs/PROJECT_TREE.md`
- 쓰러스터 구현 상세: `docs/ROV_THRUSTER_IMPL_GUIDE_KR.md`
