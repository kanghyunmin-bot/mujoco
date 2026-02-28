# UUV MuJoCo v2.2 Project Tree (SITL/QGC 전용)

```text
v2.2/
├── VERSION.txt
├── README.md
├── requirements.txt
├── install_deps_ubuntu.sh
├── launch_competition_sim.sh
├── run_urdf_full.py
├── ros2_bridge.py
├── urdf_full_scene.xml
├── sim_profiles.json
├── thruster_tune.json
├── thruster_params.json
├── scripts/
│   └── ros2_to_qgc_video.py
├── docker/
│   └── entrypoint.sh
├── assets/
│   └── urdf_full/meshes_split/*
├── validation/
└── docs/
    ├── PROJECT_TREE.md
    ├── CODE_GUIDE.md
    └── ROV_THRUSTER_IMPL_GUIDE_KR.md
```

## File-by-File

| Path | Purpose | Key Inputs | Key Outputs |
|---|---|---|---|
| `run_urdf_full.py` | SITL/QGC 제어 메인 런타임 | MAVLink/JSON 서보 입력, `sim_profiles.json`, `thruster_*.json` | MuJoCo 제어 루프, 센서 발행 |
| `ros2_bridge.py` | ROS2 I/O 브리지 | MuJoCo 센서/카메라, `/cmd_vel` | `/imu/data`, `/dvl/*`, `/depth`, `/stereo/*` |
| `urdf_full_scene.xml` | 모델/쓰러스터/센서 배치 | 메쉬/site/actuator 정의 | MuJoCo 모델 |
| `sim_profiles.json` | 물리 프로파일 | profile name | 부력/감쇠/힘 제한 |
| `thruster_tune.json` | actuator 방향 보정 | thruster name | allocator 방향 일치 |
| `thruster_params.json` | 쓰러스터 개별 gain | thruster name | 추력 크기 보정 |
| `launch_competition_sim.sh` | 운영 런처 | `--sitl*`, `--ros2*` 옵션 | 통합 실행 |
| `scripts/ros2_to_qgc_video.py` | ROS2 영상 -> QGC UDP 브리지 | `/stereo/*` | QGC 영상 스트림 |
| `install_deps_ubuntu.sh` | 의존성 설치 | apt/pip | 실행 환경 |
| `docker/entrypoint.sh` | 컨테이너 진입점 | ROS2 setup | 지정 커맨드 실행 |

## Runtime Artifacts

실행 중 로그/산출물은 주로 아래 경로에 생성됩니다.

- `~/antigravity/log/full_stack_*/*.log`
- `v2.2/validation/*` (운영 로그 보관 용도)
