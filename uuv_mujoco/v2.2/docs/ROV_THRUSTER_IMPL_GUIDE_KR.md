# ROV 쓰러스터 구현 가이드북 (v2.2, SITL/QGC 전용)

마지막 갱신: 2026-02-28  
범위: `uuv_mujoco/v2.2`

이 문서는 **ArduSub SITL + QGroundControl 제어 경로**에 직접 관련된 쓰러스터 구현만 다룹니다.  
`캘리브레이션`, `키보드 수동 조작`, `로컬 조이스틱 수동 조작` 내용은 제외합니다.

---

## 1) 한 줄 요약

- 추진은 `urdf_full_scene.xml` actuator/site + `run_urdf_full.py`의 SITL 경로(`SERVO_OUTPUT_RAW -> thr_target -> data.ctrl`)로 구현됩니다.
- 운영 기본은 `--sitl-servo-source mavlink` + `--sitl-direct-thrusters`입니다.

---

## 2) SITL/QGC 기준 구현 순서

### 2.1 모델(요소) 단계

1. 쓰러스터 site 정의  
파일: `uuv_mujoco/v2.2/urdf_full_scene.xml:65-72`

```xml
      <site name="thr_ver_lf" pos="0.3297 0.228406 0.052" />
      <site name="thr_ver_lr" pos="0.0097 0.228406 0.052" />
      <site name="thr_ver_rf" pos="0.3297 0.22841 -0.417" />
      <site name="thr_ver_rr" pos="0.0097 0.22841 -0.417" />
      <site name="thr_yaw_lf" pos="0.26359 0.12 0.00029211" />
      <site name="thr_yaw_lr" pos="0.08309 0.12 0.00065541" />
      <site name="thr_yaw_rf" pos="0.26374 0.12 -0.36613" />
      <site name="thr_yaw_rr" pos="0.083645 0.12 -0.3655" />
```

2. actuator + gear 정의  
파일: `uuv_mujoco/v2.2/urdf_full_scene.xml:134-143`

```xml
    <motor name="ver_lf" site="thr_ver_lf" gear="0 1 0 0 0 0" ctrlrange="-80 80" />
    <motor name="ver_lr" site="thr_ver_lr" gear="0 1 0 0 0 0" ctrlrange="-80 80" />
    <motor name="ver_rf" site="thr_ver_rf" gear="0 1 0 0 0 0" ctrlrange="-80 80" />
    <motor name="ver_rr" site="thr_ver_rr" gear="0 1 0 0 0 0" ctrlrange="-80 80" />

    <motor name="yaw_lf" site="thr_yaw_lf" gear="0.735142 0 -0.677913 0 0 0" ctrlrange="-80 80" />
    <motor name="yaw_lr" site="thr_yaw_lr" gear="0.707105 0 0.707108 0 0 0" ctrlrange="-80 80" />
    <motor name="yaw_rf" site="thr_yaw_rf" gear="0.707105 0 0.707108 0 0 0" ctrlrange="-80 80" />
    <motor name="yaw_rr" site="thr_yaw_rr" gear="0.705343 0 -0.708867 0 0 0" ctrlrange="-80 80" />
```

3. 센서 기준점(브리지 발행 기준)  
파일: `uuv_mujoco/v2.2/urdf_full_scene.xml:84-92`, `:148-153`

---

### 2.2 SITL 입력 경로 초기화

1. SITL 관련 CLI 인자  
파일: `uuv_mujoco/v2.2/run_urdf_full.py:278-461`

```python
    parser.add_argument("--sitl", action="store_true", ...)
    parser.add_argument("--sitl-servo-source", type=str, default="json", choices=("json", "mavlink"), ...)
    parser.add_argument("--sitl-mavlink-endpoint", type=str, default="udpin:0.0.0.0:14660", ...)
    ...
    parser.add_argument("--sitl-direct-thrusters", dest="sitl_direct_thrusters", action="store_true", ...)
    parser.add_argument("--no-sitl-direct-thrusters", dest="sitl_direct_thrusters", action="store_false", ...)
    parser.set_defaults(sitl_direct_thrusters=True)
```

2. 런처에서 SITL 기본값 강제  
파일: `uuv_mujoco/v2.2/launch_competition_sim.sh:232-243`, `:265-270`

```bash
    if ! extra_arg_present "--sitl-servo-source"; then
        EXTRA_ARGS+=(--sitl-servo-source mavlink)
        echo "[launch] SITL mode: defaulting servo source to mavlink."
    fi

    if ! extra_arg_present "--sitl-mavlink-endpoint"; then
        EXTRA_ARGS+=(--sitl-mavlink-endpoint "udpin:0.0.0.0:14660")
    fi
```

3. SITL 모드에서 내부 depth/stabilize 비활성화  
파일: `uuv_mujoco/v2.2/run_urdf_full.py:1583-1591`

```python
    if args.sitl:
        if args.imu_stabilize or args.depth_hold:
            print(
                "[simulation] SITL mode: internal stabilization/depth-hold are disabled; "
                "control authority is delegated to ArduPilot.",
                flush=True,
            )
        args.imu_stabilize = False
        args.depth_hold = False
```

---

### 2.3 SITL -> 쓰러스터 명령 변환

1. 수평/수직 쓰러스터 순서 고정  
파일: `uuv_mujoco/v2.2/run_urdf_full.py:1466-1469`

```python
    yaw_names = ["yaw_lf", "yaw_lr", "yaw_rf", "yaw_rr"]
    horiz_order = ["yaw_rf", "yaw_lf", "yaw_rr", "yaw_lr"]
    ver_names = ["ver_lf", "ver_lr", "ver_rf", "ver_rr"]
```

2. SITL direct-thruster 기본 매핑/부호  
파일: `uuv_mujoco/v2.2/run_urdf_full.py:415-424`

```python
    parser.add_argument(
        "--sitl-servo-map",
        type=str,
        default="yaw_rf,yaw_lf,yaw_rr,yaw_lr,ver_rf,ver_lf,ver_rr,ver_lr",
        ...
    )
    parser.add_argument(
        "--sitl-servo-signs",
        type=str,
        default="1,1,1,1,-1,-1,-1,-1",
        ...
    )
```

3. PWM 정규화  
파일: `uuv_mujoco/v2.2/run_urdf_full.py:1804-1808`

```python
    def sitl_pwm_to_norm(pwm: int) -> float:
        if pwm <= 0 or pwm == 65535:
            return 0.0
        return float(np.clip((float(pwm) - 1500.0) / 400.0, -1.0, 1.0))
```

4. direct-thruster 수신 핸들러  
파일: `uuv_mujoco/v2.2/run_urdf_full.py:1919-1927`

```python
            def on_sitl_servo_packet(pwm_values: list[int]) -> None:
                for thr_name in all_thruster_names:
                    sitl_servo_cmd_norm[thr_name] = 0.0
                for idx, thr_name in enumerate(raw_map):
                    if idx >= len(pwm_values):
                        break
                    norm = sitl_pwm_to_norm(int(pwm_values[idx])) * servo_signs[idx]
                    sitl_servo_cmd_norm[thr_name] = float(np.clip(norm, -1.0, 1.0))
                sitl_servo_last_wall["value"] = time.monotonic()
```

5. (옵션) mixer-inverse 해석  
파일: `uuv_mujoco/v2.2/run_urdf_full.py:1773-1802`, `:1836-1857`

```python
    sitl_motor_mix = {
        "vectored": np.array([...], dtype=np.float64),
        "vectored_6dof": np.array([...], dtype=np.float64),
    }
    sitl_motor_mix_pinv = {
        name: np.linalg.pinv(mat) for name, mat in sitl_motor_mix.items()
    }
```

```python
                cmd = pinv @ motors
                cmd = np.clip(np.nan_to_num(cmd, nan=0.0, posinf=1.0, neginf=-1.0), -1.0, 1.0)
                sitl_mixer_cmd["roll"] = float(cmd[0])
                sitl_mixer_cmd["pitch"] = float(cmd[1])
                sitl_mixer_cmd["yaw"] = float(cmd[2])
                sitl_mixer_cmd["throttle"] = float(cmd[3])
                sitl_mixer_cmd["forward"] = float(cmd[4])
                sitl_mixer_cmd["lateral"] = float(-cmd[5])
```

---

### 2.4 할당기 + 힘 적용

1. 수평 allocator  
파일: `uuv_mujoco/v2.2/run_urdf_full.py:1691-1717`

```python
    def build_horizontal_allocator() -> None:
        ...
        tau = np.cross(r, fdir)
        alloc[:, i] = np.array([fdir[0], fdir[2], tau[1]], dtype=np.float64)
        ...
        horiz_pinv = np.linalg.pinv(alloc / row_scale[:, None])
```

2. 수직 allocator  
파일: `uuv_mujoco/v2.2/run_urdf_full.py:2024-2043`

```python
    def build_vertical_allocator() -> None:
        ...
        tau = np.cross(r, fdir * thruster_force_max)
        alloc[:, i] = np.array([tau[0], tau[2]], dtype=np.float64)
        ...
        vert_pinv = np.linalg.pinv(alloc)
```

3. `thr_target -> data.ctrl`  
파일: `uuv_mujoco/v2.2/run_urdf_full.py:2098-2109`

```python
    def update_thruster_forces(_dt: float) -> None:
        for name in all_thruster_names:
            ...
            if perf_cfg.get("active") and perf_cfg.get("force").size > 0:
                force = pwm_to_force_from_perf(target_norm) * gain
            else:
                force = target_norm * thruster_force_max * gain
            data.ctrl[aid] = float(np.clip(force, lo, hi))
```

4. 유체력 적용  
파일: `uuv_mujoco/v2.2/run_urdf_full.py:2124-2156`

```python
    def apply_underwater_wrench(_dt: float) -> None:
        ...
        depth = water_surface_z - float(com[2])
        frac = np.clip((depth + half_height) / (2.0 * half_height), 0.0, 1.0)
        buoy = rho * g * neutral_volume * frac * buoyancy_scale
        ...
        data.xfrc_applied[base_id, 0:3] -= lin_drag_coeff * lin_vel
        data.xfrc_applied[base_id, 3:6] -= ang_drag_coeff * ang_vel
```

---

### 2.5 메인 루프(SITL 전용 분기)

파일: `uuv_mujoco/v2.2/run_urdf_full.py:2720-2801`

```python
        if sitl_direct_thrusters:
            now = time.monotonic()
            stale = (
                sitl_servo_last_wall["value"] <= 0.0
                or (now - sitl_servo_last_wall["value"]) > sitl_servo_timeout_s
            )
            if stale:
                for name in all_thruster_names:
                    thr_target[name] = 0.0
            else:
                ...
            update_thruster_forces(model.opt.timestep)
            update_propeller_visuals(model.opt.timestep if not is_paused else 0.0)
            apply_underwater_wrench(model.opt.timestep if not is_paused else 0.0)
            if not is_paused:
                mujoco.mj_step(model, data)
```

핵심:
- SITL 서보 수신이 stale이면 즉시 `thr_target=0` fail-safe
- 수신 정상일 때만 thruster 명령 반영

---

## 3) SITL/QGC 트러블슈팅 체크리스트

1. MAVLink 서보 입력 경로 확인
- `launch_competition_sim.sh` 실행 로그에 아래가 있어야 함
- `SITL mode: using servo source mavlink (SERVO_OUTPUT_RAW).`
- `SITL MAVLink servo input enabled: endpoint=udpin:0.0.0.0:14660`

2. direct-thruster 맵/부호 확인
- 기본 맵: `yaw_rf,yaw_lf,yaw_rr,yaw_lr,ver_rf,ver_lf,ver_rr,ver_lr`
- 기본 부호: `1,1,1,1,-1,-1,-1,-1`
- 불일치 시 `--sitl-servo-map`, `--sitl-servo-signs`만 우선 수정

3. 서보 stale fail-safe 여부 확인
- `run_step()`에서 stale 조건 시 `thr_target` 전체 0
- 통신 끊김 시 즉시 thrust 0이 정상 동작

4. 내부 제어 충돌 확인
- SITL 모드에서는 내부 `--imu-stabilize`, `--depth-hold`가 자동 비활성화
- 자세/깊이 제어는 ArduSub/QGC 측 권한

---

## 4) 핵심 파일과 라인 참조 (SITL/QGC 한정)

- `uuv_mujoco/v2.2/urdf_full_scene.xml`
- 쓰러스터 site: `65-72`
- actuator: `134-143`

- `uuv_mujoco/v2.2/run_urdf_full.py`
- SITL 인자: `278-461`
- 수평/수직 이름순서: `1466-1469`
- 수평 allocator: `1691-1717`
- SITL 매핑/믹서: `1773-1941`
- 힘 적용: `2098-2109`
- 유체력: `2124-2156`
- SITL 메인 분기: `2731-2801`

- `uuv_mujoco/v2.2/launch_competition_sim.sh`
- SITL 기본 강제 설정: `232-277`
- 실행 엔트리: `303-308`

---

## 5) 운영용 기본 실행 (SITL/QGC)

```bash
cd /home/khm/antigravity/mujoco/uuv_mujoco/v2.2
./launch_competition_sim.sh --sitl --images --force-clean
```

필요 시 입력 스케일만 조정:

```bash
./launch_competition_sim.sh --sitl --images --force-clean \
  --sitl-servo-scale 1.0 \
  --sitl-roll-scale 0.45 \
  --sitl-pitch-scale 0.45
```

---

## 6) 팀 브리핑 한 문장

- "`v2.2`의 SITL/QGC 쓰러스터 제어는 `SERVO_OUTPUT_RAW`를 direct-thruster 또는 mixer-inverse로 해석해 `thr_target`을 만들고, allocator/force clamp를 거쳐 `data.ctrl`에 반영한 뒤 부력/감쇠를 적용하는 고정 파이프라인이다."
