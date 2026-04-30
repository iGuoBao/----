# AGENTS.md

本文件为本仓库的 AI 编码代理快速指引。
目标：先读关键入口，做最小改动，优先保证可编译与策略行为稳定。

## 1. 快速事实

- 平台：STM32F103RC（Keil MDK-ARM 工程）
- 主入口：Core/Src/main.c
- 策略入口：Core/Application/startup_strategy.c -> startup_strategy_run()
- 铲车主流程：Core/Application/plan.c -> plan_loader_patrol_loop()
- 工程分层：Core/Application（策略与控制）/ Core/Src 与 Core/Inc（HAL 初始化与中断）/ Drivers（厂商库）

## 2. 构建与验证

- IDE 工程：MDK-ARM/HY4car.uvprojx
- 典型命令行构建（若本机安装 UV4）：
  - UV4 -r MDK-ARM/HY4car.uvprojx -t HY4car
- 构建输出位于：MDK-ARM/HY4car/
- 本仓库没有标准化单元测试；改动后至少做：
  - 编译通过
  - 启动策略可进入目标分支
  - 关键 route 命令链不提前退出且不死循环

重要：Keil 工程是“显式文件列表”。新增 .c/.h 后，必须同步更新 MDK-ARM/HY4car.uvprojx 对应 Group，否则会链接失败。

## 3. 代码边界（先看这里再改）

- 定位：Core/Application/GlobalLocalization.c/.h
  - 对外：GlobalLoc_Init / GlobalLoc_Periodic / GlobalLoc_GetPose
- 规划：Core/Application/astar.c/.h
  - 对外：AStar_FindPath / AStar_SetObstacle / AStar_GetMap
- 路径转译：Core/Application/translate_route_cmd.c/.h
  - 对外：TranslateRouteCmd_Generate* 系列
- 执行动作：Core/Application/action.c/.h
  - 对外：route / forward / turn_left / turn_right / turn_around
- 策略状态机：Core/Application/loader_strategy.c/.h
  - 对外：LoaderStrategy_Init / LoaderStrategy_RunOnce / LoaderStrategy_RunLoop

改动原则：
- 如果需求是“策略行为变化”，优先改 loader_strategy 或 plan，不要先动 action 底层。
- 如果需求是“路径命令不对”，优先改 translate_route_cmd，不要先动 A*。
- 如果需求是“定位跳格/回退异常”，先查 GlobalLocalization 的路口触发与速度符号逻辑。

## 4. 项目特有约定

- 坐标约定（GlobalLocalization）：
  - X 正方向为前进，Y 正方向为左，yaw 逆时针为正
- route 命令字符（action.c）：
  - 转向：L / R / A
  - 前进：'1'..'9'
  - 执行动作：K/O/t/d/f/b 等
- translate_route_cmd 输出不自动追加 S；调用侧决定是否追加结束动作。
- 方案优先级：先参数/阈值微调，再考虑结构改动。

## 5. 高频坑点（改前必读）

- forward 前置初始化不能省：
  - 需要在直行控制入口完成 mpu6050_pid_reset + mpu6050_sevenway_init
  - 否则可能出现方向校正失效
- 运动保护是按需启用：
  - Action_EnableMotionGuard 只在 loader_strategy 中开启
  - forward 超时会置 motion fault，route 会提前退出
- 回退时定位跳格风险：
  - 路口触发下，回退不应按前进方向推进网格
  - 修改定位逻辑时要特别检查 linear_velocity_mm_s 的符号分支
- 特殊边前进计数规则：
  - TranslateRouteCmd_AddEdgeRouteUnits 允许单边 route_units=0
  - 但整段直线累计为 0 会返回 ERR_ZERO_FORWARD
- UART3 接收链敏感：
  - 中断处理、NVIC、单字节重装填必须保持完整链路

## 6. 推荐工作流（最小改动）

1. 先定位问题归属层（定位/规划/转译/执行/策略）。
2. 只改一层，避免跨层联动重构。
3. 若新增源文件，同步更新 MDK-ARM/HY4car.uvprojx。
4. 编译验证后，再做一次策略主路径冒烟验证。

## 7. 参考文档（链接，不复制）

- 工程约束与改动原则：[note/CLAUDE.md](note/CLAUDE.md)
- 策略层概览：[note/策略层实现.md](note/策略层实现.md)
- 启动策略说明：[note/启动策略.md](note/启动策略.md)
- 路径转译说明：[note/路径转译命令模块的功能分析.md](note/%E8%B7%AF%E5%BE%84%E8%BD%AC%E8%AF%91%E5%91%BD%E4%BB%A4%E6%A8%A1%E5%9D%97%E7%9A%84%E5%8A%9F%E8%83%BD%E5%88%86%E6%9E%90.md)
- 铲车流程规划草案：[note/铲车策略.md](note/%E9%93%B2%E8%BD%A6%E7%AD%96%E7%95%A5.md)
