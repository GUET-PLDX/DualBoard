# DualBoard

`DualBoard` 是面向 RoboMaster 双主控的定频 CAN 业务帧模块。它保留 LibXR Topic 作为模块边界：上层模块仍发布/订阅 `chassis_cmd`、`yawmotor_angle`、`pitchmotor_angle`、`gimbal_euler`、`launcher_ref` 和 `dualboard_chassis_mode`，但板间物理传输不再转发完整 Topic packet，而是使用固定 8 字节 Classic CAN 帧。

## 角色

- `DualBoard<DualBoardRole::GIMBAL, Omni>`：云台板，发送运动控制帧，接收底盘板裁判帧，完整重组后发布本地 `sentry_ref`。
- `DualBoard<DualBoardRole::CHASSIS, Omni>`：底盘板，接收运动控制帧；监听底盘本地 `sentry_ref`，通过固定 CAN2 业务帧发送给云台板。

默认 CAN ID 沿用参考工程方向习惯：

- 云台板：`tx_id = 0x312`，`rx_id = 0x311`
- 底盘板：`tx_id = 0x311`，`rx_id = 0x312`

云台到 底盘 的 ID 分配：

| CAN ID | 周期 | DLC | 内容 |
|---|---:|---:|---|
| `tx_id + 0x00` | 10 ms | 8 | `CMD::ChassisCMD` 的 `x/y/z/self_define` 和底盘模式 |
| `tx_id + 0x10` | 10 ms | 8 | 云台 yaw/pitch 机械角，`int16` 定点编码 |
| `tx_id + 0x1f` | 按需 | 8 | Sentry decision；云台板每条 pending decision 至少成功发送 5 次 |
| `tx_id + 0x20` | 10 ms | 8 | 云台 roll/pitch/yaw 姿态摘要，`int16` 定点编码 |

底盘到云台的 ID 分配：

| CAN ID | 周期 | DLC | 内容 |
|---|---:|---:|---|
| `tx_id + 0x00` | 20 ms | 8 | 发射模块使用的热量上限、冷却、当前热量、弹速和机器人等级 |
| `tx_id + 0x10` | 10 ms | 8 | `chassis_gyro.z()`，按 900 LSB/(rad/s) 定点编码，含有效标志 |

云台侧 decision 帧使用 `tx_id + 0x1f`（默认云台 ID 为 `0x331`），避开底盘已有的
`0x327` 裁判帧和云台姿态帧 `0x332`。帧为 `SentryDecisionFrame`：版本、序列号、
有效位、状态、购弹增量、远程购弹次数和复活标志均固定在一个 Classic CAN 帧内。

裁判系统固定帧使用底盘发送基址 `0x311`：

| CAN ID | 内容 |
|---|---|
| `0x313` | 比赛状态 |
| `0x314` | 场地事件 |
| `0x315-0x317` | 全场 HP（2026 友敌相对字段） |
| `0x318-0x31A` | 机器人性能体系 |
| `0x31B-0x31C` | 功率和热量 |
| `0x31D-0x31E` | Buff |
| `0x31F-0x320` | 允许发弹量 |
| `0x323` | RFID |
| `0x324` | 伤害状态 |
| `0x325-0x326` | 本机位置 |
| `0x327` | 裁判链路与源有效位 |
| `0x328` | 轮速遥测版本、序列、底盘采样时间低 32 位、全局状态 |
| `0x329` | 轮 0/1 的 Q8.8 rad/s 与轮状态 |
| `0x32A` | 可选底盘诊断 `vx/vy` mm/s、`wz` mrad/s |
| `0x32B` | 轮 2/3 的 Q8.8 rad/s 与轮状态 |

多帧业务每帧携带 1 字节序号和 7 字节数据。重组器在全部分片到齐后才发布，
混合序号会重置，20 ms 未完成会过期，重复完成帧不会重复发布。

轮速遥测保持 Omni 几何顺序（0: `0x204`，1: `0x201`，2: 反向
`0x202`，3: `0x203`），不按 CAN ID 排序。meta、pair01、pair23 同序列
到齐后才发布 `chassis_wheel_telemetry`；diagnostic 可缺失。Q8.8 饱和会设置
`ENCODING_SATURATED` 并清对应轮 `FRESH`。云台侧使用独立 50 ms stream
watchdog，首次超时立即发布 invalid，持续超时时每 100 ms 发布一次 invalid
heartbeat；该状态不修改 DualBoard 通用 `online_`。

新增链路固定为每个 100 Hz semantic sample 发送 4 帧，即约 400 frame/s。
标准 11-bit Classic CAN、8-byte payload 每帧名义约 111 bit，按最坏位填充及
帧间隔估算约 130 bit，增量线负载约 52 kbit/s：1 Mbit/s 总线约 5.2%，
500 kbit/s 总线约 10.4%。这是静态预算；真实 arbitration、错误重发和总线利用率
必须在硬件门禁中测量，不能由该估算替代。

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
    sentry_buy_bullet_num_topic_name: sentry_buy_bullet_num
    sentry_remote_buy_bullet_times_topic_name: sentry_remote_buy_bullet_times
    sentry_remote_buy_hp_times_topic_name: sentry_remote_buy_hp_times
    sentry_buy_resurrection_topic_name: sentry_buy_resurrection
    sentry_state_topic_name: sentry_state
    wheel_telemetry_topic_name: chassis_wheel_telemetry
```

底盘板需要把 `ROLE` 改为 `DualBoardRole::CHASSIS`，CAN ID 对调，并传入 `chassis: '@&chassis'`。

## Topic 边界

Gimbal -> Chassis：

- `chassis_cmd`，类型 `CMD::ChassisCMD`
- `yawmotor_angle`，类型 `float`
- `pitchmotor_angle`，类型 `float`
- `gimbal_euler`，类型 `LibXR::EulerAngle<float>`
- `dualboard_chassis_mode`，类型 `uint32_t`
- `sentry_buy_bullet_num`，类型 `uint16_t`
- `sentry_remote_buy_bullet_times`，类型 `uint8_t`
- `sentry_remote_buy_hp_times`，类型 `uint8_t`
- `sentry_buy_resurrection`，类型 `bool`
- `sentry_state`，类型 `uint8_t`

Chassis -> Gimbal：

- `launcher_ref`，类型 `Referee::LauncherPack`，仅回填发射模块实际消费的摘要字段
- `chassis_gyro`，类型 `Eigen::Matrix<float, 3, 1>`，底盘侧 BMI088 输入
- `chassis_gyro_z`，类型 `float`，云台侧单发布者 Topic；无效帧或完整链路失联时发布零
- `sentry_ref`，类型 `Referee::RobotGameRefereePack`，底盘侧为 CAN2 输入，云台侧为完整重组后的本地输出
- `chassis_wheel_telemetry`，类型 `ChassisWheelTelemetry`，底盘侧为 100 Hz
  semantic 输入，云台侧为固定 CAN 帧完整重组后的输出

云台五个 decision source Topic 的回调只把 `DecisionUpdate` 放入 32 深度
`MPMCQueue`；协议线程聚合 pending frame（购弹增量饱和到 `2047`，远程次数饱和到
`15`，状态和复活值 latest-wins）。active frame 在 retry 期间不可被新回调覆盖，
decision callback update queue 满时记录原子 drop counter，并由协议线程限频告警。
每轮 2 ms owner iteration 最多处理 32 个 update，避免并发持续补充队列时阻塞 retry、
运动帧、失联检查和告警服务。五个 decision Topic 均保持 single-publisher 属性，兼容
`SentryProtocol` 与 `DualBoard` 的任一构造顺序，也保证 ISR Topic publish 不进入 mutex。

底盘端先验证版本、有效位和字段范围，再用序列号去重。新序列只发布一次购弹增量，
并按远程请求次数发布 `uint8_t{1}`；复活和状态各发布一次 level 值。重复帧只刷新
decision freshness，100 ms 没有有效 decision 后允许发送端重启并复用序列号。该链路
是 at-least-once（发送成功五次），接收端去重不提供持久化 exactly-once 语义。
decision freshness 与运动链路的 `last_rx_time_ms_`、`online_` 和安全状态完全分离，
缺失 decision 不会放宽运动失联保护。

`GetEvent()` 暴露给 `EventBinder`。云台侧收到 `Omni::ChassisMode` 事件后更新周期控制帧中的模式字段；如果构造参数注入了可选的 `cmd`，云台侧还会监听 `CMD_EVENT_LOST_CTRL` 和 `CMD_EVENT_START_CTRL`，将本地底盘模式和底盘命令缓存降级为 `RELAX`/零命令，使下一帧控制帧不再携带断联前的旧模式。底盘侧仅在模式变化或链路恢复时调用 `chassis->GetEvent().Active(mode)`，避免 10 ms 周期帧反复重置底盘 PID。

## 失联保护

底盘侧超过 `offline_timeout_ms` 未收到控制帧时，会发布零 `chassis_cmd`、零云台角、零姿态，并强制底盘进入 `RELAX`；离线期间收到的 angle/attitude 帧会被丢弃，直到新的 control frame 恢复链路。云台侧由底盘 MotionFrame 或 launcher feedback 刷新同一条链路状态；超过 `offline_timeout_ms` 未收到任一帧时，会发布零 `launcher_ref` 和零 `chassis_gyro_z`，避免继续使用旧裁判摘要或旧底盘角速度。该检查由 `DualBoard` 自身 2 ms 周期线程执行，不依赖全局 `monitor_sleep_ms`。

## 限制

- 当前版本只支持 Classic CAN 8 字节帧。
- `rx_buffer_size` 和 `tx_slot_count` 继续保留为构造参数，以保持 YAML/xrobot 构造契约兼容。
- 裁判数据只走上述固定 CAN2 业务帧；`SharedTopicClient` 不传输 `sentry_ref`。
