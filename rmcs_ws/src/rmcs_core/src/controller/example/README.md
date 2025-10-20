# RMCS Example Components

这是一个简单的示例，展示如何创建和使用 RMCS 组件。

## 组件说明

### SineCosineGenerator（组件A）

**功能**：生成正弦和余弦波形

**输入接口**：
- `/predefined/timestamp` - 时间戳
- `/predefined/update_count` - 更新计数

**输出接口**：
- `/example/sine` - sin(ωt)
- `/example/cosine` - cos(ωt)

**配置参数**：
- `omega` (double) - 角频率 ω (rad/s)

### SineCosineAdder（组件B）

**功能**：将正弦和余弦值相加

**输入接口**：
- `/example/sine` - 来自组件A
- `/example/cosine` - 来自组件A

**输出接口**：
- `/example/sum` - sin(ωt) + cos(ωt)

## 运行示例

```bash
# 1. 编译
build-rmcs

# 2. 运行
ros2 launch rmcs_bringup rmcs.launch.py robot:=example-test

# 3. 查看输出（在另一个终端）
ros2 topic echo /example/sine
ros2 topic echo /example/cosine
ros2 topic echo /example/sum
```

## 数学说明

- 组件A输出：sin(ωt) 和 cos(ωt)
- 组件B输出：sin(ωt) + cos(ωt) = √2 · sin(ωt + π/4)
- 更新频率：1000Hz (1ms 一次)
- 当 ω = 2π 时，周期为 1 秒

## 组件串联图

```
PredefinedMsgProvider
    ↓ /predefined/update_count
SineCosineGenerator (A)
    ↓ /example/sine
    ↓ /example/cosine
SineCosineAdder (B)
    ↓ /example/sum
ValueBroadcaster
    → ROS2 Topics (可视化)
```

