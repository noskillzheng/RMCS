# 使用 Foxglove 可视化示例组件

## 前置条件

- ✅ 已安装 Foxglove Studio
- ✅ RMCS 示例组件已编译
- ✅ ROS2 环境已配置

---

## 步骤一：启动 RMCS 示例

```bash
# 在终端1中运行
cd /workspaces/RMCS
source install/setup.zsh  # 或 setup.bash

# 启动示例
ros2 launch rmcs_bringup rmcs.launch.py robot:=example-test
```

你应该看到类似的输出：
```
[INFO] [sine_cosine_generator]: SineCosineGenerator initialized with omega = 6.28 rad/s
[INFO] [sine_cosine_adder]: SineCosineAdder initialized
```

---

## 步骤二：验证话题发布

在另一个终端检查话题是否正常发布：

```bash
# 列出所有话题
ros2 topic list

# 你应该看到：
# /example/sine
# /example/cosine
# /example/sum
```

查看话题数据：
```bash
# 查看 sine 波形
ros2 topic echo /example/sine

# 查看数据类型
ros2 topic info /example/sine
# Type: std_msgs/msg/Float64
```

---

## 步骤三：启动 Foxglove Bridge（推荐方式）

### 方法1：使用 Foxglove Bridge（推荐）

```bash
# 安装 Foxglove Bridge（如果还没安装）
sudo apt install ros-humble-foxglove-bridge

# 启动 bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Foxglove Bridge 会在 `ws://localhost:8765` 启动 WebSocket 服务器。

### 方法2：使用 Rosbridge（备选）

```bash
# 安装 rosbridge（如果还没安装）
sudo apt install ros-humble-rosbridge-suite

# 启动 rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## 步骤四：在 Foxglove Studio 中连接

1. **打开 Foxglove Studio**

2. **连接到数据源**
   - 点击 "Open connection"
   - 选择 "Foxglove WebSocket"
   - URL: `ws://localhost:8765` (如果使用 Foxglove Bridge)
   - 或 `ws://localhost:9090` (如果使用 Rosbridge)
   - 点击 "Open"

3. **验证连接**
   - 在左侧面板应该能看到可用的话题
   - 找到 `/example/sine`, `/example/cosine`, `/example/sum`

---

## 步骤五：创建可视化面板

### 方案A：使用 Plot 面板（推荐）

1. **添加 Plot 面板**
   - 点击顶部 "+" 按钮
   - 选择 "Plot"

2. **配置三条曲线**
   - 点击 "Add series"
   - 添加以下三个系列：
     - `/example/sine` → 选择 `data` 字段 → 颜色：蓝色
     - `/example/cosine` → 选择 `data` 字段 → 颜色：红色
     - `/example/sum` → 选择 `data` 字段 → 颜色：绿色

3. **调整显示设置**
   - Y轴范围：-2 到 2
   - 时间窗口：5 秒或 10 秒
   - 勾选 "Show legend"

### 方案B：使用 State Transitions（数值显示）

1. 添加三个 "Indicator" 面板
2. 分别配置为：
   - `/example/sine` → 显示当前 sin(ωt) 值
   - `/example/cosine` → 显示当前 cos(ωt) 值
   - `/example/sum` → 显示当前和值

---

## 期望的可视化效果

你应该看到：

1. **蓝色曲线 (sine)**：
   - 标准正弦波
   - 振幅：-1 到 1
   - 周期：1 秒（ω = 2π）

2. **红色曲线 (cosine)**：
   - 标准余弦波（相位领先 90°）
   - 振幅：-1 到 1
   - 周期：1 秒

3. **绿色曲线 (sum)**：
   - sin + cos = √2·sin(ωt + π/4)
   - 振幅：约 -1.414 到 1.414
   - 周期：1 秒
   - 相位：领先 sine 45°

---

## 调试技巧

### 如果看不到数据

1. **检查 RMCS 是否运行**
   ```bash
   ros2 node list
   # 应该看到 rmcs_executor
   ```

2. **检查话题发布频率**
   ```bash
   ros2 topic hz /example/sine
   # 应该显示约 1000 Hz
   ```

3. **检查数据内容**
   ```bash
   ros2 topic echo /example/sine --once
   # 应该显示：data: <某个数值>
   ```

4. **检查 Bridge 状态**
   ```bash
   # 如果使用 Foxglove Bridge
   ros2 node list | grep foxglove
   ```

### 如果波形异常

1. **验证数学计算**
   - sin²(ωt) + cos²(ωt) = 1 (总是成立)
   - 可以添加第四个图表验证

2. **检查时间戳**
   - 确保 update_count 在递增
   - 检查 ω 参数是否正确

---

## 高级可视化配置

### 添加更多信息

可以在配置文件中添加更多输出：

```yaml
# example-test.yaml
value_broadcaster:
  ros__parameters:
    forward_list:
      - /example/sine
      - /example/cosine
      - /example/sum
      - /predefined/update_rate      # 添加频率监控
      - /predefined/update_count     # 添加计数监控
```

### 创建自定义布局

在 Foxglove 中：
1. 排列好所有面板
2. 点击顶部 "Save layout"
3. 命名为 "RMCS Example"
4. 下次可以直接加载

### 导出数据

1. 在 Foxglove 中点击 "Record"
2. 选择要记录的话题
3. 保存为 MCAP 格式（可回放）

---

## 常见问题

### Q: Foxglove 连接失败？

**A**: 检查防火墙和端口：
```bash
# 检查端口是否监听
netstat -an | grep 8765  # Foxglove Bridge
netstat -an | grep 9090  # Rosbridge
```

### Q: 话题列表为空？

**A**: 确保 Bridge 和 RMCS 都在运行，并且在同一个 ROS_DOMAIN_ID

### Q: 数据更新很慢？

**A**: 
- 检查 RMCS 是否以 1000Hz 运行
- Foxglove 可能需要调整刷新率（默认 60fps）

### Q: 想修改波形频率？

**A**: 修改配置文件中的 omega：
```yaml
sine_cosine_generator:
  ros__parameters:
    omega: 12.566370614  # 2Hz (2 * 2π)
    # omega: 3.141592654  # 0.5Hz (π)
    # omega: 62.83185307  # 10Hz (20π)
```

---

## 参考资料

- [Foxglove Studio 文档](https://foxglove.dev/docs)
- [Foxglove Bridge GitHub](https://github.com/foxglove/ros-foxglove-bridge)
- [ROS2 Visualization 指南](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

---

**提示**：可以同时使用 `rqt_plot` 作为备选可视化工具：
```bash
rqt_plot /example/sine/data /example/cosine/data /example/sum/data
```

