# DualBoard

`DualBoard` 是面向 RoboMaster 双主控的定频 CAN 业务帧模块。它保留 LibXR Topic 作为模块边界：上层模块仍发布/订阅 `chassis_cmd`、`yawmotor_angle`、`pitchmotor_angle`、`gimbal_euler`、`launcher_ref` 和 `dualboard_chassis_mode`，但板间物理传输不再转发完整 Topic packet，而是使用固定 8 字节 Classic CAN 帧。

## 角色

- `DualBoard<DualBoardRole::GIMBAL, Omni>`：云台板，缓存本地 `chassis_cmd`、云台机械角、云台姿态和底盘模式，按 10 ms 周期发给底盘板；接收底盘板的发射裁判摘要并发布为 `launcher_ref`。
- `DualBoard<DualBoardRole::CHASSIS, Omni>`：底盘板，接收云台板固定帧并重新发布为本地 LibXR Topic；监听本地 `launcher_ref`，按 20 ms 周期压缩回传给云台板。

默认 CAN ID 沿用参考工程方向习惯：

- 云台板：`tx_id = 0x312`，`rx_id = 0x311`
- 底盘板：`tx_id = 0x311`，`rx_id = 0x312`

云台到 底盘 的 ID 分配：

| CAN ID | 周期 | DLC | 内容 |
|---|---:|---:|---|
| `tx_id + 0x00` | 10 ms | 8 | `CMD::ChassisCMD` 的 `x/y/z/self_define` 和底盘模式 |
| `tx_id + 0x10` | 10 ms | 8 | 云台 yaw/pitch 机械角，`int16` 定点编码 |
| `tx_id + 0x20` | 10 ms | 8 | 云台 roll/pitch/yaw 姿态摘要，`int16` 定点编码 |

底盘到云台的 ID 分配：

| CAN ID | 周期 | DLC | 内容 |
|---|---:|---:|---|
| `tx_id + 0x00` | 20 ms | 8 | 发射模块使用的热量上限、冷却、当前热量、弹速和机器人等级 |
| `tx_id + 0x10` | 10 ms | 8 | `chassis_gyro.z()`，按 900 LSB/(rad/s) 定点编码，含有效标志 |

## 构造参数

```yaml
- id: dual_board
  name: DualBoard
  template_args:
    ROLE: DualBoardRole::GIMBAL
    ChassisType: Omni
  constructor_args:
    can_bus_name: can2
    tx_id: 0x312
    rx_id: 0x311
    rx_buffer_size: 256
    tx_slot_count: 8
    offline_timeout_ms: 100
    chassis: '@nullptr'
    mode_topic_name: dualboard_chassis_mode
    cmd: '@&cmd'
```

底盘板需要把 `ROLE` 改为 `DualBoardRole::CHASSIS`，CAN ID 对调，并传入 `chassis: '@&chassis'`。

## Topic 边界

Gimbal -> Chassis：

- `chassis_cmd`，类型 `CMD::ChassisCMD`
- `yawmotor_angle`，类型 `float`
- `pitchmotor_angle`，类型 `float`
- `gimbal_euler`，类型 `LibXR::EulerAngle<float>`
- `dualboard_chassis_mode`，类型 `uint32_t`

Chassis -> Gimbal：

- `launcher_ref`，类型 `Referee::LauncherPack`，仅回填发射模块实际消费的摘要字段
- `chassis_gyro`，类型 `Eigen::Matrix<float, 3, 1>`，底盘侧 BMI088 输入
- `chassis_gyro_z`，类型 `float`，云台侧单发布者 Topic；无效帧或完整链路失联时发布零
- `sentry_ref`，类型 `Referee::RobotGameRefereePack`，当前仅保留 Topic 兼容，不进入 CAN2 运动链路
- `sentry_state`，类型 `uint8_t`，当前仅保留 Topic 兼容

`GetEvent()` 暴露给 `EventBinder`。云台侧收到 `Omni::ChassisMode` 事件后更新周期控制帧中的模式字段；如果构造参数注入了可选的 `cmd`，云台侧还会监听 `CMD_EVENT_LOST_CTRL` 和 `CMD_EVENT_START_CTRL`，将本地底盘模式和底盘命令缓存降级为 `RELAX`/零命令，使下一帧控制帧不再携带断联前的旧模式。底盘侧仅在模式变化或链路恢复时调用 `chassis->GetEvent().Active(mode)`，避免 10 ms 周期帧反复重置底盘 PID。

## 失联保护

底盘侧超过 `offline_timeout_ms` 未收到控制帧时，会发布零 `chassis_cmd`、零云台角、零姿态，并强制底盘进入 `RELAX`；离线期间收到的 angle/attitude 帧会被丢弃，直到新的 control frame 恢复链路。云台侧由底盘 MotionFrame 或 launcher feedback 刷新同一条链路状态；超过 `offline_timeout_ms` 未收到任一帧时，会发布零 `launcher_ref` 和零 `chassis_gyro_z`，避免继续使用旧裁判摘要或旧底盘角速度。该检查由 `DualBoard` 自身 2 ms 周期线程执行，不依赖全局 `monitor_sleep_ms`。

## 限制

- 当前版本只支持 Classic CAN 8 字节帧。
- `rx_buffer_size` 和 `tx_slot_count` 继续保留为构造参数，以保持 YAML/xrobot 构造契约兼容。
- 完整 `sentry_ref` 大包不再通过 CAN2 透传；若上位机后续确实需要完整裁判摘要，应设计独立低频链路或专用摘要字段。
