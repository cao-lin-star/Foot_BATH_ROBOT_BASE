# 基站命令 B0~B8、C0、C1 执行逻辑

本文说明基站项目 `Foot_bath_robot_base_V1` 收到命令帧时的代码执行路径�?
## 总入�?
基站通信层在 `Hardware/uart_comm.c` 中处理桶�?主控透传帧：

```c
if ((frame[3] >= 0xB0U) && (frame[3] <= 0xBFU))
{
  Base_HandleCommand(frame);
}
```

因此当前基站只会�?`B0~BF` 命令交给 `Base_HandleCommand()` 执行业务逻辑。`C0`、`C1` 不在这个范围内，当前基站项目不会执行它们�?
## B0~B8 命令

主要代码位置：`Hardware/system_monitor.c` �?`Base_HandleCommand(const uint8_t *frame)`�?
| 命令 | 名称 | 执行逻辑 | 主要代码 |
| --- | --- | --- | --- |
| `B0` | 关机 | 停止输出，进入关机状态；电机回到排水位�?| `Base_StartPositionedAction(..., BASE_POSITIONED_OUTPUT_OFF, MOTOR_POSITION_DRAIN_0)` |
| `B1` | 待机 | 停止当前动作并回到待机状态�?| `Base_FinishToStandby()` |
| `B2` | 自动注水 | 保存目标水位 `frame[5]`、目标温�?`frame[6]`；打开 `WATER_IN`；按当前温度控制 `EN_HEAT`；水位达标后关闭进水和加热�?| `Base_HandleAutoFillFrame(frame)` |
| `B3` | 自动清洁 | 启动自动清洁流程：排水、清洁液+进水混合喷淋、排水、热水喷淋、排水、烘干。清洁喷淋阶段同时打开 `CLEAN_PUMP` �?`WATER_IN`�?| `Base_StartAutoClean()` |
| `B4` | 强制排水 | 切到强制排水动作，打开排水输出，默认持�?`60s`�?| `Base_StartPositionedAction(BASE_ACTION_FORCE_DRAIN, ..., BASE_POSITIONED_OUTPUT_DRAIN, ...)` |
| `B5` | 单独清洁液喷�?| 使用 `frame[17]` 作为分钟数，执行清洁�?进水混合喷淋，同时打开 `CLEAN_PUMP` �?`WATER_IN`�?| `Base_StartPositionedAction(BASE_ACTION_SINGLE_CLEAN_SPRAY, ..., Base_MinToMs(base_clean_spray_min), ...)` |
| `B6` | 单独清水/热水喷淋 | 使用 `frame[18]` 作为分钟数，执行热水喷淋�?| `Base_StartPositionedAction(BASE_ACTION_SINGLE_CLEAR_SPRAY, ..., Base_MinToMs(base_clear_spray_min), ...)` |
| `B7` | 单独烘干 | 使用 `frame[19]` 作为分钟数，执行烘干�?| `Base_StartPositionedAction(BASE_ACTION_SINGLE_DRY, ..., Base_DryMinToMs(base_dry_min), ...)` |
| `B8` | 自检 | 关闭输出，清除故障，进入自检状态�?| `Base_StartPositionedAction(..., BASE_POSITIONED_OUTPUT_SELF_CHECK, MOTOR_POSITION_DRAIN_0)` |

## B2 自动注水细节

`B2` 的执行由三组函数组成�?
```c
Base_HandleAutoFillFrame(frame);
Base_StartAutoFill(frame);
Base_UpdateAutoFill(frame[5], frame[6]);
```

执行逻辑�?
1. 首次收到 `B2` 命令时，`Base_StartAutoFill()` 关闭其它输出，保存目标值�?2. 打开 `WATER_IN`�?3. 如果当前温度低于目标温度，打开 `EN_HEAT`；否则关闭加热�?4. 后续收到桶体状态帧时，`Base_UpdateAutoFill()` 用当前水�?`frame[5]` 判断是否达标�?5. 当前水位达到目标水位后，`Base_StopAutoFill()` 关闭 `WATER_IN` �?`EN_HEAT`，回到待机状态�?
## C0、C1 当前行为

当前基站项目没有 `C0`、`C1` 的处理逻辑�?
原因是通信入口只转�?`B0~BF`�?
```c
if ((frame[3] >= 0xB0U) && (frame[3] <= 0xBFU))
{
  Base_HandleCommand(frame);
}
```

所以：

| 命令 | 当前行为 |
| --- | --- |
| `C0` | 不进�?`Base_HandleCommand()`，当前基站不执行动作 |
| `C1` | 不进�?`Base_HandleCommand()`，当前基站不执行动作 |

如果后续需要基站支�?`C0/C1`，需要先�?`uart_comm.c` 增加 `0xC0~0xCF` 的分发入口，再在 `system_monitor.c` 增加对应系统命令处理函数�?
## 排水退出条件

B3/B4/B5/B6 中的排水阶段逻辑为：排水输出打开后先启动 5min 保护倒计时；如果收到桶体实时水量为 0L，则改为 0L 后继续排水 60s，倒计时结束后切换到下一状态。这样可避免没有收到 0L 帧时基站一直保持排水状态。
