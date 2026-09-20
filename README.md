# DualBoard

模块实现、控制帧、裁判分片编解码和哨兵决策协议均位于 `DualBoard.hpp`。其他模块通过 `#include "DualBoard.hpp"` 使用共享类型；主机协议测试位于 `tests/`。

`DualBoard` 是面向 RoboMaster 双主控的定频 CAN 业务帧模块。它保留 LibXR Topic 作为模块边界：上层模块仍发布/订阅 `chassis_cmd`、`yawmotor_angle`、`pitchmotor_angle`、`gimbal_euler`、`launcher_ref` 和 `dualboard_chassis_mode`，但板间物理传输不再转发完整 Topic packet，而是使用固定 8 字节 Classic CAN 帧。

## 角色

- `DualBoard<DualBoardRole::GIMBAL, Omni>`：云台板，发送运动控制帧，接收底盘板裁判帧，完整重组后发布本地 `sentry_ref`。
- `DualBoard<DualBoardRole::CHASSIS, Omni>`：底盘板，接收运动控制帧；监听底盘本地 `sentry_ref`，通过固定 CAN2 业务帧发送给云台板。

角色专用 Topic、缓存和协议状态由编译期选择的 `GimbalState` / `ChassisState` 保存。五个决策发送队列和裁判重组缓冲仅在云台角色构造。链路超时和总线故障检查由 2 ms 协议线程统一执行。

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
| `0x330` | 底盘 AHRS yaw：`int16 yaw_q`、`uint8 valid`、`uint8 sequence`、`uint32 sample_time_ms` |

多帧业务每帧携带 1 字节序号和 7 字节数据。重组器在全部分片到齐后才发布，
混合序号会重置，20 ms 未完成会过期，重复完成帧不会重复发布。

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
    offline_timeout_ms: 100
    chassis: '@nullptr'
    mode_topic_name: dualboard_chassis_mode
    cmd: '@&cmd'
    sentry_buy_bullet_num_topic_name: sentry_buy_bullet_num
    sentry_remote_buy_bullet_times_topic_name: sentry_remote_buy_bullet_times
    sentry_remote_buy_hp_times_topic_name: sentry_remote_buy_hp_times
    sentry_buy_resurrection_topic_name: sentry_buy_resurrection
    sentry_state_topic_name: sentry_state
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

云台五个 decision source Topic 各挂一个 `LibXR::Topic::QueuedSubscriber` 和深度 32
的 `SPSCQueue`。协议线程 round-robin 聚合 pending frame（购弹增量饱和到 `2047`，远程
次数饱和到 `15`，状态和复活值 latest-wins）。active frame 在 retry 期间不可被新回调
覆盖。`QueuedSubscriber` 入队前若对应 SPSC 队列已满，原子 drop counter 加一，并由协议
线程限频告警。每轮 2 ms owner iteration 最多处理 32 个 update，避免并发持续补充队列时
阻塞 retry、运动帧、失联检查和告警服务。五个 decision Topic 均保持 single-publisher
属性，兼容 `SentryProtocol` 与 `DualBoard` 的任一构造顺序，也保证 ISR Topic publish
不进入 mutex。

底盘端先验证版本、有效位和字段范围，再用序列号去重。新序列只发布一次购弹增量，
并按远程请求次数发布 `uint8_t{1}`；复活和状态各发布一次 level 值。重复帧只刷新
decision freshness，100 ms 没有有效 decision 后允许发送端重启并复用序列号。该链路
是 at-least-once（发送成功五次），接收端去重不提供持久化 exactly-once 语义。
decision freshness 与运动链路的 `last_rx_time_ms_`、`online_` 和安全状态完全分离，
缺失 decision 不会放宽运动失联保护。

`GetEvent()` 暴露给 `EventBinder`。云台侧把左拨杆模式和导航模式分开放：`EventBinder` 写入遥控槽，`nav_chassis_mode` 写入导航槽。发送控制帧时按 `CMD` 是否自动档选择输出；遥控失联时输出 `RELAX` 和零指令，但不清掉遥控槽。云台侧只监听 `CMD_EVENT_LOST_CTRL` 做上述强制输出。底盘侧仅在模式变化或链路恢复时调用 `chassis->GetEvent().Active(mode)`，避免 10 ms 周期帧反复重置底盘 PID。

## 失联保护

底盘侧超过 `offline_timeout_ms` 未收到控制帧时，会发布零 `chassis_cmd`、零云台角、零姿态，并强制底盘进入 `RELAX`；离线期间收到的 angle/attitude 帧会被丢弃，直到新的 control frame 恢复链路。云台侧由底盘 MotionFrame 或 launcher feedback 刷新同一条链路状态；超过 `offline_timeout_ms` 未收到任一帧时，会发布零 `launcher_ref` 和零 `chassis_gyro_z`，避免继续使用旧裁判摘要或旧底盘角速度。`DualBoard` 同时订阅 `LibXR::CAN::Type::ERROR` 并轮询 `GetErrorState()`：bus-off 或 error-passive 立即按失联处理，不等待业务帧超时。该检查由 `DualBoard` 自身 2 ms 周期线程执行，不依赖全局 `monitor_sleep_ms`。

## 限制

- 当前版本只支持 Classic CAN 8 字节帧。
- `rx_buffer_size` 是 CAN RX `SPSCQueue` 容量。CAN 发送队列深度由 `User/libxr_config.yaml` 的 `CAN.CAN2.queue_size` 配置。
- 板内 Topic 默认按单发布者创建；只有 `chassis_motion_state` 使用其契约里的 multi-publisher 属性。
- 裁判数据只走上述固定 CAN2 业务帧；`SharedTopicClient` 不传输 `sentry_ref`。
