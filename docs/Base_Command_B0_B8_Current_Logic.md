# 基站 B0-B8 命令当前代码逻辑说明

本文档记录当前基站工程中 `B0` 到 `B8` 命令的执行流程，依据当前源码：

- `Hardware/system_monitor.c`
- `Hardware/system_monitor.h`
- `Hardware/uart_comm.c`
- `Core/Inc/main.h`
- `Hardware/motor_control.h`

当前代码使用非阻塞状态机：命令入口只记录动作、状态、目标电机位置和待执行输出，真正的输出打开、计时推进和阶段切换由 `SystemMonitor_TaskProcess()` 周期执行。

## 1. 关键时间参数

| 宏 | 当前值 | 含义 | 使用位置 |
| --- | ---: | --- | --- |
| `BASE_DEFAULT_DRAIN_MS` | 60s | 启动排水动作时传入的默认值；当前排水到位后会被覆盖为最长排水保护时间 | `Base_StartAutoClean()`、`B4`、单项喷淋后的排水 |
| `BASE_AUTO_FILL_PRIME_MS` | 5s | 自动注水收到 `B2` 后，先只打开进水阀 5 秒用于管道排空/预冲 | `Base_StartAutoFill()`、`Base_UpdateAutoFillPrime()` |
| `BASE_AUTO_FILL_TIMEOUT_MS` | 15min | 自动注水最长保护时间；从第一次进入 `B2` 自动注水开始计时 | `Base_StartAutoFill()`、`Base_CheckAutoFillTimeout()` |
| `BASE_DRAIN_AFTER_EMPTY_MS` | 10s | 排水阶段收到桶体水量为 0L 后，继续排水 10 秒再进入下一阶段 | `Base_UpdateDrainEmptyTimer()` |
| `BASE_DRAIN_MAX_MS` | 10min | 排水阶段未收到 0L 时的最长排水保护时间 | `Base_CompletePositionedAction()` |
| `BASE_AUTO_CLEAN_CLEANER_MS` | 5s | 自动清洁喷淋阶段开始后，清洁液和水一起喷淋的时间 | `Base_UpdateAutoCleanSprayPattern()` |
| `BASE_AUTO_CLEAN_WAIT_MS` | 10s | 清洁液+水喷淋结束后的等待时间 | `Base_UpdateAutoCleanSprayPattern()` |
| `BASE_LEVEL_BOARD_TIMEOUT_MS` | 5s | 缺液控制板通信超时判定时间 | `Base_IsLevelBoardTimeout()` |

注意：排水动作虽然启动时常传入 `BASE_DEFAULT_DRAIN_MS`，但喷淋杆到位并调用 `Base_CompletePositionedAction()` 后，`BASE_POSITIONED_OUTPUT_DRAIN` 会将 `base_pending_duration_ms` 覆盖为 `BASE_DRAIN_MAX_MS`。所以当前排水实际最长是 10 分钟；若桶体水量先到 0L，则转为 10 秒后结束。

## 2. 命令帧字段

`Base_HandleCommand(const uint8_t *frame)` 处理 `frame[3]` 为 `0xB0..0xBF` 的基站命令。每次有效命令都会先更新：

| 字段 | 代码变量 | 含义 |
| --- | --- | --- |
| `frame[3]` | `cmd` / `base_last_cmd` | 当前命令字 |
| `frame[17]` | `base_clean_spray_min` | 清洁液喷淋时间，单位分钟 |
| `frame[18]` | `base_clear_spray_min` | 清水/热水喷淋时间，单位分钟 |
| `frame[19]` | `base_dry_min` | 烘干时间，单位分钟 |

`B2` 自动注水额外使用：

| 字段 | 代码变量 | 含义 |
| --- | --- | --- |
| `frame[16]` | `base_auto_fill_target_water` | 目标水位 |
| `frame[6]` | `base_auto_fill_target_temp` | 目标温度 |
| `frame[20]` | `base_auto_fill_med1_sec` | 药液泵 1 投放时间，单位秒 |
| `frame[21]` | `base_auto_fill_med2_sec` | 药液泵 2 投放时间，单位秒 |

桶体实时帧 `frame[3] == 0x00` 由 `Base_UpdateBucketRealtimeData()` 处理：

- `frame[5]`：当前桶体水位，写入 `base_bucket_current_water`。
- `frame[6]`：当前温度，传给 `Base_UpdateAutoFill(current_water, current_temp)` 用于自动注水加热控制。

## 3. 输出引脚和逻辑输出

当前 `Core/Inc/main.h` 定义如下，本文按当前代码为准：

| 逻辑名 | MCU 引脚 | 宏 | 1 表示 |
| --- | --- | --- | --- |
| `DRY_FAN` | PA1 | `DRY_FAN_GPIO_Port`, `DRY_FAN_Pin` | 烘干风机打开 |
| `EN_HEAT` | PA0 | `EN_HEAT_GPIO_Port`, `EN_HEAT_Pin` | 加热打开 |
| `WATER_IN` | PB5 | `WATER_IN_GPIO_Port`, `WATER_IN_Pin` | 进水打开 |
| `WATER_OUT` | PB4 | `WATER_OUT_GPIO_Port`, `WATER_OUT_Pin` | 排水打开 |
| `MED_PUMP1` | PB12 | `MED_PUMP1_GPIO_Port`, `MED_PUMP1_Pin` | 药液泵 1 打开 |
| `MED_PUMP2` | PB13 | `MED_PUMP2_GPIO_Port`, `MED_PUMP2_Pin` | 药液泵 2 打开 |
| `CLEAN_PUMP` | PB14 | `CLEAN_PUMP_GPIO_Port`, `CLEAN_PUMP_Pin` | 清洁液泵打开 |
| `EN_IR` | PB3 | `EN_IR_GPIO_Port`, `EN_IR_Pin` | 红外输出打开 |
| `SPARE_SW` | PB15 | `SPARE_SW_GPIO_Port`, `SPARE_SW_Pin` | 备用输出打开 |

## 4. 状态码

状态码写入状态上报 `frame[23]`。

| 状态 | 值 | 含义 |
| --- | ---: | --- |
| `BASE_STATUS_POWER_ON` | `0x00` | 上电 |
| `BASE_STATUS_SELF_CHECK` | `0x01` | 自检 |
| `BASE_STATUS_OFF` | `0x02` | 关机 |
| `BASE_STATUS_STANDBY` | `0x03` | 待机 |
| `BASE_STATUS_AUTO_FILL` | `0x04` | 自动注水 |
| `BASE_STATUS_FILL_KEEP_WARM` | `0x05` | 注水完成且目标温度非 0 后的等待状态 |
| `BASE_STATUS_FILL_WAIT_DRAIN` | `0x06` | 注水完成且目标温度为 0 后的等待排水状态 |
| `BASE_STATUS_CLEAN_SPRAY` | `0x07` | 自动清洁清洁喷淋 |
| `BASE_STATUS_CLEAN_DRAIN1` | `0x08` | 自动清洁第一次清洁后排水 |
| `BASE_STATUS_CLEAR_SPRAY` | `0x09` | 清水/热水喷淋 |
| `BASE_STATUS_CLEAR_DRAIN2` | `0x0A` | 清水/热水喷淋后排水 |
| `BASE_STATUS_CLEAN_DRY` | `0x0B` | 自动清洁烘干 |
| `BASE_STATUS_FORCE_DRAIN` | `0x0C` | 强制排水 |
| `BASE_STATUS_SINGLE_CLEAN_SPRAY` | `0x0D` | 单独清洁液喷淋 |
| `BASE_STATUS_SINGLE_CLEAR_SPRAY` | `0x0E` | 单独清水/热水喷淋 |
| `BASE_STATUS_SINGLE_DRY` | `0x0F` | 单独烘干 |

`frame[24]` 由 `SystemMonitor_GetSubStatus()` 填充：若 `base_sub_status != 0` 则直接返回；否则状态大于等于 `BASE_STATUS_AUTO_FILL` 时返回当前动作剩余分钟数；其它状态返回进入当前状态后的秒数，最大 255。

## 5. 公共动作框架

### 5.1 `Base_StartPositionedAction()`

函数签名：

```c
static void Base_StartPositionedAction(BaseAction_t action,
                                       uint8_t protocol_status,
                                       uint32_t duration_ms,
                                       BasePositionedOutput_t output,
                                       MotorPosition_t position)
```

作用：启动一个需要喷淋杆定位后再执行输出的动作。

步骤：

1. 关闭 `WATER_IN`、`WATER_OUT`、`EN_HEAT`、`DRY_FAN`、`EN_IR`、`MED_PUMP1`、`MED_PUMP2`、`CLEAN_PUMP`、`SPARE_SW`。
2. 调用 `Base_StopMedicineDosing()` 停止药液泵计时。
3. 清理桶体循环请求、自动注水状态、待执行输出状态。
4. 设置 `base_action = action`。
5. 调用 `Base_SetMainStatus(protocol_status)` 设置状态和进入状态的 tick。
6. 设置 `base_pending_output = output`、`base_pending_duration_ms = duration_ms`。
7. 调用 `Motor_MoveToPosition(position)`，异步移动喷淋杆。

### 5.2 `Base_CompletePositionedAction()`

当 `SystemMonitor_TaskProcess()` 发现 `base_pending_output != NONE` 且 `Motor_IsBusy() == 0` 时调用。

根据 `base_pending_output` 执行具体输出：

| output | 函数 | 输出行为 | 计时 |
| --- | --- | --- | --- |
| `BASE_POSITIONED_OUTPUT_DRAIN` | `Base_ApplyDrain()` | 关进水/加热/清洁泵，打开排水 | 计时被设为 `BASE_DRAIN_MAX_MS`，即 10min |
| `BASE_POSITIONED_OUTPUT_CLEAN_SPRAY` | `Base_ApplyCleanSpray()` 或 `Base_ApplyCleanClearWater()` | 自动清洁先清洁液+进水喷淋；单独清洁液喷淋直接清洁液+进水 | 使用传入 `duration_ms` |
| `BASE_POSITIONED_OUTPUT_HOT_SPRAY` | `Base_ApplyHotSpray()` | 进水+加热，清洁泵关 | 使用传入 `duration_ms` |
| `BASE_POSITIONED_OUTPUT_DRY` | `Base_ApplyDry()` | 只开风机，不开加热 | 使用传入 `duration_ms` |
| `BASE_POSITIONED_OUTPUT_STANDBY` | 内联处理 | 停电机、关灯、状态待机 | 不计时 |
| `BASE_POSITIONED_OUTPUT_OFF` | 内联处理 | 停电机、关灯、状态关机 | 不计时 |
| `BASE_POSITIONED_OUTPUT_SELF_CHECK` | 内联处理 | 清错误码、状态自检、绿灯 | 不计时 |

完成定位并应用输出后，若需要计时：

```c
base_action_start_tick = HAL_GetTick();
base_action_deadline = base_action_start_tick + base_pending_duration_ms;
```

## 6. B0-B8 命令流程

## B0：关机命令

命令：`BASE_CMD_OFF = 0xB0`

入口：`Base_HandleCommand()` 的 `case BASE_CMD_OFF`

调用：

```c
Base_StartPositionedAction(BASE_ACTION_IDLE,
                           BASE_STATUS_OFF,
                           0UL,
                           BASE_POSITIONED_OUTPUT_OFF,
                           MOTOR_POSITION_DRAIN_0);
```

流程：

1. 关闭所有基站输出。
2. 停止药液泵计时。
3. 清理自动注水、桶体循环、待执行输出状态。
4. 喷淋杆移动到 `MOTOR_POSITION_DRAIN_0`。
5. 到位后 `Base_CompletePositionedAction()` 处理 `BASE_POSITIONED_OUTPUT_OFF`。
6. 调用 `Motor_Stop()`、`ColorLight_Off()`。
7. `base_action = BASE_ACTION_IDLE`。
8. 状态设为 `BASE_STATUS_OFF / 0x02`。

时间：无业务计时；只有喷淋杆定位时间。

## B1：待机命令

命令：`BASE_CMD_STANDBY = 0xB1`

入口：`Base_HandleCommand()` 的 `case BASE_CMD_STANDBY`

调用：

```c
Base_FinishToStandby();
```

`Base_FinishToStandby()` 内部调用：

```c
Base_StartPositionedAction(BASE_ACTION_IDLE,
                           BASE_STATUS_STANDBY,
                           0UL,
                           BASE_POSITIONED_OUTPUT_STANDBY,
                           MOTOR_POSITION_DRAIN_0);
```

流程：

1. 关闭所有基站输出。
2. 停止药液泵计时。
3. 清理自动注水、桶体循环、待执行输出状态。
4. 喷淋杆移动到 `0°`。
5. 到位后进入 `BASE_POSITIONED_OUTPUT_STANDBY`。
6. 停止电机、关闭氛围灯。
7. 状态设为 `BASE_STATUS_STANDBY / 0x03`。

时间：无业务计时；只有喷淋杆定位时间。

## B2：自动注水命令

命令：`BASE_CMD_AUTO_FILL = 0xB2`

入口：`Base_HandleCommand()` 的 `case BASE_CMD_AUTO_FILL`

调用：

```c
Base_HandleAutoFillFrame(frame);
```

### B2 使用参数

| 参数 | 含义 |
| --- | --- |
| `frame[16]` | 目标水位 |
| `frame[6]` | 目标温度 |
| `frame[20]` | 药液泵 1 投放秒数 |
| `frame[21]` | 药液泵 2 投放秒数 |

### 初次进入自动注水

当 `base_auto_fill_active == 0` 时，调用：

```c
Base_StartAutoFill(frame);
```

流程：

1. 若已经完成过自动注水，且 `frame[16] != 0`，且已有桶体实时水位数据，且当前水位已经 `>= frame[16]`：
   - 关闭 `WATER_IN`。
   - 关闭 `EN_HEAT`。
   - 清理自动注水预冲和药液待投字段。
   - 直接返回，不重新注水。
2. 调用 `Base_AllOutputsOff()` 关闭所有基站输出并清理动作状态。
3. 设置：
   - `base_auto_fill_active = 1`
   - `base_auto_fill_target_water = frame[16]`
   - `base_auto_fill_target_temp = frame[6]`
   - `base_auto_fill_completed = 0`
   - `base_auto_fill_prime_deadline = HAL_GetTick() + BASE_AUTO_FILL_PRIME_MS`
   - `base_auto_fill_timeout_deadline = HAL_GetTick() + BASE_AUTO_FILL_TIMEOUT_MS`
   - `base_auto_fill_med1_sec = frame[20]`
   - `base_auto_fill_med2_sec = frame[21]`
   - `base_action = BASE_ACTION_AUTO_FILL`
   - 主状态 `BASE_STATUS_AUTO_FILL / 0x04`
4. 打开 `WATER_IN=1`。
5. 保持 `EN_HEAT=0`。
6. 设置氛围灯 `ColorLight_SetRgbw(0, 40, 80, 0)`。

### 自动注水前 5 秒管道排空

函数：`Base_UpdateAutoFillPrime(now)`

调用位置：`SystemMonitor_TaskProcess()` 每轮调用。

逻辑：

1. 如果自动注水未激活，或 `base_auto_fill_prime_deadline == 0`，直接返回。
2. 如果当前 tick 未到 `base_auto_fill_prime_deadline`，直接返回。
3. 到 5 秒后：
   - `base_auto_fill_prime_deadline = 0`
   - 调用 `Base_StartMedicineDosing()` 启动药液泵计时
   - 保持 `WATER_IN=1`
   - 如果目标温度非 0，则 `EN_HEAT=1`，否则 `EN_HEAT=0`

5 秒预冲期间，实时水位/温度帧仍会进入 `Base_UpdateAutoFill()`，但只会保持进水并强制 `EN_HEAT=0`，不会提前加热和投药。

### 药液泵投放

函数：`Base_StartMedicineDosing()`

逻辑：

- 药液泵 1：`base_auto_fill_med1_sec != 0` 且药液 1 不缺液时打开，deadline 为当前 tick 加秒数。
- 药液泵 2：`base_auto_fill_med2_sec != 0` 且药液 2 不缺液时打开，deadline 为当前 tick 加秒数。
- 周期函数 `Base_UpdateMedicineDosing(now)` 到时关闭药液泵；如果执行中检测到对应药液缺液，也立即关闭对应药液泵。

### 自动注水运行中控制

桶体实时帧 `frame[3] == 0x00` 进入 `Base_UpdateBucketRealtimeData(frame)`，再调用：

```c
Base_UpdateAutoFill(frame[5], frame[6]);
```

其中：

- `frame[5]` 是当前水位。
- `frame[6]` 是当前温度。

`Base_UpdateAutoFill()` 逻辑：

1. 自动注水未激活则返回。
2. 若当前水位 `>= base_auto_fill_target_water`：
   - 调用 `Base_StopAutoFill()`。
   - 关闭进水和加热。
   - 若目标温度非 0，状态变为 `BASE_STATUS_FILL_KEEP_WARM / 0x05`。
   - 若目标温度为 0，状态变为 `BASE_STATUS_FILL_WAIT_DRAIN / 0x06`。
3. 若未到目标水位：
   - 保持 `WATER_IN=1`。
   - 若仍在 5 秒预冲期，保持 `EN_HEAT=0`。
   - 预冲结束后，当前温度 `< 目标温度` 时加热打开，否则关闭。

### 自动注水 15 分钟超时

函数：`Base_CheckAutoFillTimeout(now)`

调用位置：`SystemMonitor_TaskProcess()`，在 `Base_UpdateAutoFillPrime()` 之前。

逻辑：

1. 仅当 `base_auto_fill_active != 0` 且 `base_auto_fill_timeout_deadline != 0` 时检查。
2. 从初次 `B2` 自动注水开始计时 15 分钟。
3. 超时后：
   - `base_auto_fill_active = 0`
   - 清理目标水位、目标温度、预冲 deadline、超时 deadline、药液待投秒数
   - 关闭 `WATER_IN`
   - 关闭 `EN_HEAT`
   - 调用 `Base_StopMedicineDosing()` 关闭药液泵
   - 若当前 `base_action == BASE_ACTION_AUTO_FILL`，状态回到 `BASE_STATUS_STANDBY / 0x03`

重复收到 `B2` 只更新目标水位/温度；不会重置 15 分钟超时计时。

### 自动注水期间重复收到 B2

当 `base_auto_fill_active != 0` 时，`Base_HandleAutoFillFrame(frame)` 不重新启动流程，只做：

1. 更新目标水位 `frame[16]`。
2. 更新目标温度 `frame[6]`。
3. 如果仍在 5 秒预冲期，更新待投药液秒数 `frame[20]` 和 `frame[21]`。
4. 保持 `WATER_IN=1`。
5. 只有预冲结束且目标温度非 0 时才打开加热。

## B3：自动清洁命令

命令：`BASE_CMD_AUTO_CLEAN = 0xB3`

入口：`Base_HandleCommand()` 的 `case BASE_CMD_AUTO_CLEAN`

调用：

```c
Logging_Print("Auto clean start\r\n");
Base_StartAutoClean();
```

`Base_StartAutoClean()` 调用：

```c
Base_StartPositionedAction(BASE_ACTION_AUTO_DRAIN1,
                           BASE_STATUS_FORCE_DRAIN,
                           BASE_DEFAULT_DRAIN_MS,
                           BASE_POSITIONED_OUTPUT_DRAIN,
                           MOTOR_POSITION_DRAIN_0);
```

自动清洁由 `Base_AdvanceAutoClean()` 分阶段推进：

| 阶段 | action | 状态 | 函数和参数 | 喷淋杆位置 | 输出 | 时间 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | `BASE_ACTION_AUTO_DRAIN1` | `BASE_STATUS_FORCE_DRAIN / 0x0C` | `Base_StartPositionedAction(..., BASE_DEFAULT_DRAIN_MS, BASE_POSITIONED_OUTPUT_DRAIN, MOTOR_POSITION_DRAIN_0)` | 0° | `Base_ApplyDrain()` 打开 `WATER_OUT` | 到位后最长 10min；水位 0L 后再排 10s |
| 2 | `BASE_ACTION_AUTO_CLEAN_SPRAY` | `BASE_STATUS_CLEAN_SPRAY / 0x07` | `Base_StartPositionedAction(..., Base_MinToMs(base_clean_spray_min), BASE_POSITIONED_OUTPUT_CLEAN_SPRAY, MOTOR_POSITION_SPRAY_90)` | 90° | 清水/清洁液喷淋节奏 | `frame[17]` 分钟 |
| 3 | `BASE_ACTION_AUTO_DRAIN2` | `BASE_STATUS_CLEAN_DRAIN1 / 0x08` | `Base_StartPositionedAction(..., BASE_DEFAULT_DRAIN_MS, BASE_POSITIONED_OUTPUT_DRAIN, MOTOR_POSITION_DRAIN_0)` | 0° | 排水 | 到位后最长 10min；水位 0L 后再排 10s |
| 4 | `BASE_ACTION_AUTO_HOT_SPRAY2` | `BASE_STATUS_CLEAR_SPRAY / 0x09` | `Base_StartPositionedAction(..., Base_MinToMs(base_clear_spray_min), BASE_POSITIONED_OUTPUT_HOT_SPRAY, MOTOR_POSITION_SPRAY_90)` | 90° | 进水+加热 | `frame[18]` 分钟 |
| 5 | `BASE_ACTION_AUTO_DRAIN3` | `BASE_STATUS_CLEAR_DRAIN2 / 0x0A` | `Base_StartPositionedAction(..., BASE_DEFAULT_DRAIN_MS, BASE_POSITIONED_OUTPUT_DRAIN, MOTOR_POSITION_DRAIN_0)` | 0° | 排水 | 到位后最长 10min；水位 0L 后再排 10s |
| 6 | `BASE_ACTION_AUTO_DRY` | `BASE_STATUS_CLEAN_DRY / 0x0B` | `Base_StartPositionedAction(..., Base_DryMinToMs(base_dry_min), BASE_POSITIONED_OUTPUT_DRY, MOTOR_POSITION_DRAIN_0)` | 0° | 风机烘干 | `frame[19]` 分钟 |
| 7 | 结束 | `BASE_STATUS_STANDBY / 0x03` | `Base_FinishToStandby()` | 0° | 关闭输出 | 无 |

### 自动清洁喷淋阶段内部节奏

函数：`Base_UpdateAutoCleanSprayPattern(now)`

仅在 `base_action == BASE_ACTION_AUTO_CLEAN_SPRAY`、无待定位输出、且 `base_action_start_tick != 0` 时运行。

在 `frame[17]` 指定的总时长内，前 15 秒采用固定节奏，之后剩余时间单清水喷淋：

| 时间段 | 调用函数 | 输出 |
| --- | --- | --- |
| 0-5s | `Base_ApplyCleanSpray()` | `WATER_IN=1`, `CLEAN_PUMP=1`, `EN_HEAT=0` |
| 5-15s | `Base_ApplyCleanSprayWait()` | `WATER_IN=0`, `CLEAN_PUMP=0`, `EN_HEAT=0` |
| 15s 后 | `Base_ApplyCleanClearWater()` | `WATER_IN=1`, `CLEAN_PUMP=0`, `EN_HEAT=0` |

`Base_SetCleanSprayOutputs(water_on, cleaner_on)` 会在尝试打开清洁泵时检查：

- 清洁液缺液 `BASE_LEVEL_CLEAN_LOW`
- 缺液控制板通信超时 `Base_IsLevelBoardTimeout(HAL_GetTick())`

任一异常存在时，`CLEAN_PUMP` 不打开，但流程继续，不回待机。

### 自动清洁热水喷淋阶段

函数：`Base_ApplyHotSpray()`

输出：

- `WATER_OUT=0`
- `WATER_IN=1`
- `EN_HEAT=1`
- `CLEAN_PUMP=0`
- 氛围灯 `ColorLight_SetRgbw(80, 40, 0, 0)`

`Base_CompletePositionedAction()` 在 `base_action == BASE_ACTION_AUTO_HOT_SPRAY2` 时设置 `bucket_circulation_requested = 1`，通信上报 `frame[28]` 会带出给桶体。

### 自动清洁烘干阶段

函数：`Base_ApplyDry()`

当前输出：

- `WATER_IN=0`
- `WATER_OUT=0`
- `EN_HEAT=0`
- `CLEAN_PUMP=0`
- `DRY_FAN=1`
- `Motor_Stop()`

也就是说当前烘干只开风机，不开加热。

## B4：强制排水命令

命令：`BASE_CMD_FORCE_DRAIN = 0xB4`

入口：`Base_HandleCommand()` 的 `case BASE_CMD_FORCE_DRAIN`

调用：

```c
Base_StartPositionedAction(BASE_ACTION_FORCE_DRAIN,
                           BASE_STATUS_FORCE_DRAIN,
                           BASE_DEFAULT_DRAIN_MS,
                           BASE_POSITIONED_OUTPUT_DRAIN,
                           MOTOR_POSITION_DRAIN_0);
```

流程：

1. 关闭所有输出，停止药液泵计时。
2. 喷淋杆移动到 0°。
3. 到位后 `Base_CompletePositionedAction()` 调用 `Base_ApplyDrain()`。
4. 输出：`WATER_IN=0`、`EN_HEAT=0`、`CLEAN_PUMP=0`、`WATER_OUT=1`。
5. 排水计时实际设置为 `BASE_DRAIN_MAX_MS=10min`。
6. 若实时桶体水位先到 0L，则 `Base_UpdateDrainEmptyTimer()` 将截止时间改为当前时间 + `BASE_DRAIN_AFTER_EMPTY_MS=10s`。
7. 到期后，由 `SystemMonitor_TaskProcess()` 进入 `Base_FinishToStandby()`。

## B5：单独清洁液喷淋命令

命令：`BASE_CMD_CLEAN_SPRAY = 0xB5`

入口：`Base_HandleCommand()` 的 `case BASE_CMD_CLEAN_SPRAY`

调用：

```c
Base_StartPositionedAction(BASE_ACTION_SINGLE_CLEAN_SPRAY,
                           BASE_STATUS_SINGLE_CLEAN_SPRAY,
                           Base_MinToMs(base_clean_spray_min),
                           BASE_POSITIONED_OUTPUT_CLEAN_SPRAY,
                           MOTOR_POSITION_SPRAY_90);
```

参数：

- `base_clean_spray_min = frame[17]`
- 时间为 `frame[17]` 分钟

流程：

1. 关闭所有输出。
2. 喷淋杆移动到 90°。
3. 到位后 `Base_CompletePositionedAction()` 识别为非自动清洁的 `BASE_POSITIONED_OUTPUT_CLEAN_SPRAY`，调用 `Base_ApplyCleanSpray()`。
4. `Base_ApplyCleanSpray()` 调用 `Base_SetCleanSprayOutputs(1U, 1U)`：
   - `WATER_OUT=0`
   - `WATER_IN=1`
   - `EN_HEAT=0`
   - `CLEAN_PUMP=1`，但清洁液缺液或缺液板通信超时时会保持 `CLEAN_PUMP=0`
5. 到 `frame[17]` 分钟后，周期任务进入单项喷淋后置排水：

```c
Base_StartPositionedAction(BASE_ACTION_SINGLE_CLEAN_DRAIN,
                           BASE_STATUS_FORCE_DRAIN,
                           BASE_DEFAULT_DRAIN_MS,
                           BASE_POSITIONED_OUTPUT_DRAIN,
                           MOTOR_POSITION_DRAIN_0);
```

6. 后置排水到位后最长 10min；水位到 0L 后再排 10s；到期回待机。

注意：`Base_UpdateAutoCleanSprayPattern()` 只对 `BASE_ACTION_AUTO_CLEAN_SPRAY` 生效，所以 `B5` 不执行自动清洁里的 5s 清洁液+水、10s 等待、剩余单清水节奏，而是整个 `frame[17]` 时间持续尝试清洁液喷淋。

## B6：单独清水/热水喷淋命令

命令：`BASE_CMD_CLEAR_SPRAY = 0xB6`

入口：`Base_HandleCommand()` 的 `case BASE_CMD_CLEAR_SPRAY`

调用：

```c
Base_StartPositionedAction(BASE_ACTION_SINGLE_CLEAR_SPRAY,
                           BASE_STATUS_SINGLE_CLEAR_SPRAY,
                           Base_MinToMs(base_clear_spray_min),
                           BASE_POSITIONED_OUTPUT_HOT_SPRAY,
                           MOTOR_POSITION_SPRAY_90);
```

参数：

- `base_clear_spray_min = frame[18]`
- 时间为 `frame[18]` 分钟

流程：

1. 关闭所有输出。
2. 喷淋杆移动到 90°。
3. 到位后 `Base_CompletePositionedAction()` 调用 `Base_ApplyHotSpray()`。
4. 输出：
   - `WATER_OUT=0`
   - `WATER_IN=1`
   - `EN_HEAT=1`
   - `CLEAN_PUMP=0`
5. 到 `frame[18]` 分钟后，周期任务进入单项喷淋后置排水：

```c
Base_StartPositionedAction(BASE_ACTION_SINGLE_CLEAR_DRAIN,
                           BASE_STATUS_FORCE_DRAIN,
                           BASE_DEFAULT_DRAIN_MS,
                           BASE_POSITIONED_OUTPUT_DRAIN,
                           MOTOR_POSITION_DRAIN_0);
```

6. 后置排水到位后最长 10min；水位到 0L 后再排 10s；到期回待机。

## B7：单独烘干命令

命令：`BASE_CMD_DRY = 0xB7`

入口：`Base_HandleCommand()` 的 `case BASE_CMD_DRY`

调用：

```c
Base_StartPositionedAction(BASE_ACTION_SINGLE_DRY,
                           BASE_STATUS_SINGLE_DRY,
                           Base_DryMinToMs(base_dry_min),
                           BASE_POSITIONED_OUTPUT_DRY,
                           MOTOR_POSITION_DRAIN_0);
```

参数：

- `base_dry_min = frame[19]`
- 时间为 `frame[19]` 分钟

流程：

1. 关闭所有输出。
2. 喷淋杆移动到 0°。
3. 到位后 `Base_CompletePositionedAction()` 调用 `Base_ApplyDry()`。
4. 输出：
   - `WATER_IN=0`
   - `WATER_OUT=0`
   - `EN_HEAT=0`
   - `CLEAN_PUMP=0`
   - `DRY_FAN=1`
5. 到 `frame[19]` 分钟后，由 `SystemMonitor_TaskProcess()` 调用 `Base_FinishToStandby()` 回待机。

当前代码中烘干只打开风机，不打开加热。

## B8：自检命令

命令：`BASE_CMD_SELF_CHECK = 0xB8`

入口：`Base_HandleCommand()` 的 `case BASE_CMD_SELF_CHECK`

调用：

```c
Base_StartPositionedAction(BASE_ACTION_IDLE,
                           BASE_STATUS_SELF_CHECK,
                           0UL,
                           BASE_POSITIONED_OUTPUT_SELF_CHECK,
                           MOTOR_POSITION_DRAIN_0);
```

流程：

1. 关闭所有输出。
2. 停止药液泵计时。
3. 清理自动注水和桶体循环状态。
4. 喷淋杆移动到 0°。
5. 到位后 `Base_CompletePositionedAction()` 处理 `BASE_POSITIONED_OUTPUT_SELF_CHECK`：
   - `Motor_Stop()`
   - `SystemMonitor_ClearErrors()`
   - `base_action = BASE_ACTION_IDLE`
   - 主状态设为 `BASE_STATUS_SELF_CHECK / 0x01`
   - `base_sub_status = 0`
   - 氛围灯 `ColorLight_SetRgbw(0, 50, 0, 0)`

时间：无业务计时；只有喷淋杆定位时间。

## 7. 周期任务推进规则

`SystemMonitor_TaskProcess()` 每轮执行：

1. 清空 `base_err1/base_err2`，重新计算当前错误。
2. 若 `Motor_HasFault()`，设置 `BASE_ERR2_SPRAY_MOTOR`。
3. `Base_UpdateLevelErrors(now)`：根据缺液位和缺液板超时设置错误码。
4. `Base_ApplyLevelProtection()`：
   - 药液 1 缺液则关闭 `MED_PUMP1`。
   - 药液 2 缺液则关闭 `MED_PUMP2`。
   - 清洁液缺液则关闭 `CLEAN_PUMP`。
   - 当前不会因为清洁液缺液或缺液板通信异常直接中断清洁流程。
5. `Base_CheckAutoFillTimeout(now)`：自动注水 15min 超时保护。
6. `Base_UpdateAutoFillPrime(now)`：自动注水 5s 预冲结束后启动投药和加热。
7. `Base_UpdateMedicineDosing(now)`：药液泵到时或缺液后关闭。
8. `Base_UpdateDrainEmptyTimer(now)`：排水时检测桶体水位 0L 后设置 10s 后结束。
9. `Base_UpdateAutoCleanSprayPattern(now)`：自动清洁清洁喷淋阶段的 10s/5s/5s 节奏。
10. 若有待定位输出且电机不忙，则调用 `Base_CompletePositionedAction()`。
11. 若 `base_action_deadline` 到期：
    - 自动清洁动作调用 `Base_AdvanceAutoClean()` 进入下一阶段。
    - `BASE_ACTION_SINGLE_CLEAN_SPRAY` 到期后进入单独清洁后置排水。
    - `BASE_ACTION_SINGLE_CLEAR_SPRAY` 到期后进入单独清水/热水喷淋后置排水。
    - 其它动作到期后调用 `Base_FinishToStandby()`。
12. 若有软件复位请求且超过 200ms，调用 `NVIC_SystemReset()`。

## 8. 通信和掉线影响

`UART_Comm_ProcessBaseFrame()` 收到有效桶体帧时：

1. 保存完整帧到 `last_bucket_frame`。
2. 刷新 `bucket_last_rx_tick`。
3. 调用 `Base_SetBucketConnected(1U)`。
4. 若 `frame[3]` 是 `0xB0..0xBF`，调用 `Base_HandleCommand(frame)`。
5. 若 `frame[3] == 0x00`，调用 `Base_UpdateBucketRealtimeData(frame)`。

桶体 5 秒无有效帧时，`UART_Comm_CheckBucketTimeout()` 调用 `Base_SetBucketConnected(0U)`。

`Base_SetBucketConnected()` 中，如果从在线变成离线，且当前动作需要桶体协同：

```c
(bucket_circulation_requested != 0U) || (base_auto_fill_active != 0U)
```

则调用 `Base_FinishToStandby()` 回待机。

影响：

- 自动注水过程中桶体掉线会停止流程回待机。
- 自动清洁中需要桶体内循环的阶段掉线会停止流程回待机。
- 单独排水等不依赖桶体协同的阶段不会因为掉线立即停止。

## 9. 状态上报字段

`Base_BuildStatusData(frame)` 填充状态上报：

| 字段 | 当前填充值 |
| --- | --- |
| `frame[4]` | `bucket_connected` |
| `frame[16]` | 当前 `WATER_IN` 输出电平 |
| `frame[17]` | `base_clean_spray_min` |
| `frame[18]` | `base_clear_spray_min` |
| `frame[19]` | `base_dry_min` |
| `frame[20]` | `SystemMonitor_GetMedicine1RemainingSec()`，药液泵 1 剩余秒数，最大 255 |
| `frame[21]` | `SystemMonitor_GetMedicine2RemainingSec()`，药液泵 2 剩余秒数，最大 255 |
| `frame[22]` | 当前 `CLEAN_PUMP` 输出电平 |
| `frame[23]` | `base_main_status` |
| `frame[24]` | `SystemMonitor_GetSubStatus()` |
| `frame[25]` | `SystemMonitor_GetErrCode1()` |
| `frame[26]` | `SystemMonitor_GetErrCode2()` |
| `frame[27]` | 氛围灯逻辑状态：有错误为 `0x04`，烘干为 `0x03`，运行计时状态为 `0x02`，其它为 `0x00` |
| `frame[28]` | 通信层填入 `Base_IsBucketCirculationRequested()` |
| `frame[29]` | 通信层计算校验和 |

## 10. B9 补充说明

虽然本文重点是 `B0-B8`，当前代码还定义了：

```c
#define BASE_CMD_STOP_AUTO_CLEAN 0xB9U
```

`B9` 只在当前 `base_action` 属于自动清洁流程时生效：

```c
Base_StopAutoClean();
```

若正在自动清洁，打印 `Auto clean stop`，然后调用 `Base_FinishToStandby()` 回待机。