# QtMotorConsole Project Overview

## 1. 工程目标

这个工程是一个基于 Qt 的 EtherCAT 电机上位机，核心功能包括：

- 打开/关闭控制卡
- 控制电机 1 / 电机 2
- 位置模式、速度模式、力矩模式控制
- 力反馈控制
- 实时曲线显示
- 采样数据记录

从结构上看，整个项目可以分成三层：

1. `QtMotorConsole`  
   负责界面、按钮事件、曲线显示
2. `MotorController`  
   负责板卡通信、电机控制、模式切换、失能逻辑
3. `SaveData`  
   负责把采样数据写入文件

---

## 2. 文件职责

### `main.cpp`

程序入口，只负责：

- 创建 `QApplication`
- 创建主窗口 `QtMotorConsole`
- 启动 Qt 事件循环

### `QtMotorConsole.h / QtMotorConsole.cpp`

主界面层，负责：

- 初始化 UI
- 初始化绘图场景
- 绑定按钮和输入框
- 启动/停止采样线程
- 根据采样缓存更新曲线

这部分不直接承担复杂控制策略，主要是“显示层 + 交互层”。

### `MotorController.h / MotorController.cpp`

控制核心层，负责：

- 打开控制卡 `OpenCard()`
- 关闭控制卡 `CloseCard()`
- 读取位置/速度/力矩
- 位置模式、速度模式、力矩模式控制
- 力反馈线程
- 单轴/双轴失能逻辑

这里是整个工程最核心的业务层。

### `SaveData.h / SaveData.cpp`

记录层，负责：

- 开始记录 `StartRecord()`
- 停止记录 `StopRecord()`
- 把采样线程得到的数据写入文本文件 `RecordDataSample()`

它不再直接读板卡，而是只消费采样线程已经拿到的数据。

---

## 3. 当前数据流

### 控制流

按钮点击 -> `QtMotorConsole` -> `MotorController`

例如：

- 点击“执行位置”  
  -> `QtMotorConsole::setupConnections()` 中的槽函数  
  -> `PositionModeMove(axis, target, speed)`

- 点击“失能电机1”  
  -> `DisableAxis(1)`

### 采样流

控制卡打开成功 -> 启动采样线程 -> 周期性读取两台电机状态

采样线程执行流程：

1. 调用 `ReadPosition(1/2)`
2. 调用 `ReadVelocity(1/2)`
3. 调用 `ReadTorque(1/2)`
4. 保存到主窗口缓存
5. 如正在记录，则写入文件

### 显示流

UI 定时器 -> `updateWave()` -> 从缓存取最新样本 -> 刷新曲线

注意：

- UI 线程不直接读板卡
- UI 线程不直接写记录文件
- 这样可以减少界面卡顿，也更利于讲解“前台显示 / 后台采样”的结构

---

## 4. 线程设计

### 主线程

负责：

- Qt UI
- 按钮响应
- 曲线绘制

### 采样线程

负责：

- 周期读取板卡数据
- 记录数据写文件

### 力反馈线程

负责：

- 力反馈模式下的速度更新与力矩输出

这样当前工程里有三类执行上下文：

1. UI 主线程
2. 采样线程
3. 力反馈线程

---

## 5. 为什么现在的结构更容易讲解

### 之前的问题

- 构造函数里同时做了太多事
- UI、采样、记录耦合较强
- 曲线刷新和板卡读取混在一起

### 现在的结构

`QtMotorConsole` 内部已经拆成：

- `setupUiState()`  
  负责输入框限制
- `setupPlotScene()`  
  负责图形场景初始化
- `setupConnections()`  
  负责所有按钮/输入信号连接
- `resetWaveBuffers()`  
  负责切换电机时清空显示缓存
- `selectedAxis()`  
  统一获取当前选中的电机编号

这意味着你在讲的时候，可以按“初始化 -> 交互 -> 采样 -> 显示 -> 控制”来讲，而不是在一个大构造函数里跳来跳去。

---

## 6. 推荐讲解顺序

如果你要给别人介绍这个工程，推荐按下面顺序讲：

1. `main.cpp`  
   程序从哪里启动
2. `QtMotorConsole`  
   UI 如何组织，按钮如何绑定
3. `MotorController`  
   真正的控制逻辑在哪里
4. `sampleLoop()`  
   实时数据如何后台采样
5. `updateWave()`  
   曲线如何从缓存刷新
6. `SaveData`  
   为什么记录功能不会拖慢主界面
7. `DisableAxis()`  
   为什么失能逻辑要做平滑回落，而不是直接 `AxisOff`

---

## 7. 后续还可以继续优化的点

如果后面还想继续往“更规范的工程结构”走，可以考虑：

- 把日志功能单独抽成 `Logger`
- 把采样线程单独抽成 `TelemetryService`
- 把力反馈功能单独抽成 `ForceFeedbackService`
- 把板卡控制进一步封装成一个类，而不是使用较多全局状态

当前这版已经比较适合项目展示和讲解，下一步就是看你要不要继续朝“类封装更强”的方向走。
