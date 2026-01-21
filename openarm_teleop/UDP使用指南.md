# OpenArm UDP遥操作完整指南

使用UDP网络通信方案控制OpenArm机器人

## 概述

本方案使用UDP网络通信替代共享内存，实现RoboCap动作捕捉数据到OpenArm机器人的遥操作控制。

### 系统架构

```
RoboCap动捕设备 → Python UDP发送器 → UDP网络 → C++ UDP接收器 → OpenArm机器人
```

### 核心优势

✅ **跨机器部署** - RoboCap和OpenArm可以在不同机器上运行  
✅ **易于调试** - JSON格式数据，可用Wireshark等工具分析  
✅ **灵活部署** - 支持本地和远程控制  
✅ **简单易用** - 接口简洁，配置方便

## 文件说明

### C++端（OpenArm控制）

- **`src/udp_joint_receiver.hpp`** - UDP接收器类
  - 接收JSON格式的关节角度数据
  - 线程安全，非阻塞接收
  - 独立接收线程

- **`control/udp_openarm_control.cpp`** - UDP控制程序
  - 使用UDP接收器获取关节角度
  - 控制OpenArm机器人运动
  - 500Hz控制频率

### Python端（数据发送）

- **`rebocap/robocap_udp_sender.py`** - RoboCap UDP发送器
  - 从RoboCap SDK读取动捕数据
  - 转换为关节角度
  - 通过UDP发送JSON数据

- **`rebocap/test_udp_sender.py`** - 测试发送器
  - 不需要RoboCap设备
  - 发送模拟的正弦波数据
  - 用于测试和调试

- **`rebocap/start_udp_sender.sh`** - 启动脚本

## 快速开始

### 步骤1: 安装依赖

```bash
# C++ JSON解析库
sudo apt-get install nlohmann-json3-dev

# Python依赖（通常已安装）
pip install numpy
```

### 步骤2: 编译UDP控制程序

在 `openarm_teleop/CMakeLists.txt` 中添加：

```cmake
# 查找JSON库
find_package(nlohmann_json REQUIRED)

# UDP控制程序
add_executable(udp_openarm_control
    control/udp_openarm_control.cpp
)

target_include_directories(udp_openarm_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(udp_openarm_control
    # 你现有的库
    openarm_hardware
    openarm_control
    openarm_dynamics
    yaml-cpp
    
    # 添加JSON库
    nlohmann_json::nlohmann_json
    pthread
)
```

然后编译：

```bash
cd openarm_teleop
mkdir -p build && cd build
cmake ..
make udp_openarm_control
```

### 步骤3: 测试运行（无需RoboCap）

**终端1 - 启动测试发送器：**
```bash
cd rebocap
python3 test_udp_sender.py 127.0.0.1 5678
```

你会看到：
```
[    60] Sent test data | Gripper: 0.500
[   120] Sent test data | Gripper: 0.750
...
```

**终端2 - 启动控制程序：**
```bash
cd openarm_teleop/build
./udp_openarm_control /path/to/openarm.urdf right_arm can0 5678
```

### 步骤4: 使用真实RoboCap

确保RoboCap软件正在运行，然后：

**终端1 - 启动RoboCap发送器：**
```bash
cd rebocap
./start_udp_sender.sh 127.0.0.1 5678 7690
```

**终端2 - 启动控制程序：**
```bash
cd openarm_teleop/build
./udp_openarm_control /path/to/openarm.urdf right_arm can0 5678
```

## 数据格式

### UDP传输的JSON格式

```json
{
  "timestamp": 1234567890.123,
  "joints": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
  "gripper": 0.5,
  "pelvis": [0.0, 1.0, 0.0]
}
```

### 字段说明

- **timestamp** (double): RoboCap时间戳
- **joints** (array[7]): 7个关节角度（弧度）
  - Joint 0: 肩部偏航 (Shoulder yaw)
  - Joint 1: 肩部俯仰 (Shoulder pitch)
  - Joint 2: 肩部滚转 (Shoulder roll)
  - Joint 3: 肘部俯仰 (Elbow pitch)
  - Joint 4: 腕部偏航 (Wrist yaw)
  - Joint 5: 腕部俯仰 (Wrist pitch)
  - Joint 6: 腕部滚转 (Wrist roll)
- **gripper** (double): 夹爪位置 0.0-1.0
- **pelvis** (array[3]): 骨盆位置 [x, y, z]

## 性能指标

| 指标 | 本地UDP | 网络UDP |
|------|---------|---------|
| 延迟 | ~100-500µs | 1-50ms |
| 丢包率 | <0.1% | 0.1-1% |
| 数据率 | 60Hz | 60Hz |
| 包大小 | ~200字节 | ~200字节 |

对于人类遥操作（反应时间~100-200ms），UDP延迟完全可以接受。

## 命令行参数

### Python UDP发送器

```bash
python3 robocap_udp_sender.py [udp_host] [udp_port] [rebocap_port]
```

**参数说明：**
- `udp_host`: 目标IP地址（默认: 127.0.0.1）
- `udp_port`: 目标UDP端口（默认: 5678）
- `rebocap_port`: RoboCap SDK端口（默认: 7690）

**示例：**
```bash
# 本地测试
python3 robocap_udp_sender.py 127.0.0.1 5678 7690

# 远程控制
python3 robocap_udp_sender.py 192.168.1.100 5678 7690
```

### C++ UDP控制程序

```bash
./udp_openarm_control <urdf_path> [arm_side] [can_interface] [udp_port]
```

**参数说明：**
- `urdf_path`: OpenArm URDF文件路径（必需）
- `arm_side`: 手臂侧 `left_arm` 或 `right_arm`（默认: right_arm）
- `can_interface`: CAN接口名称（默认: can0）
- `udp_port`: UDP监听端口（默认: 5678）

**示例：**
```bash
./udp_openarm_control /path/to/openarm.urdf right_arm can0 5678
```

## 远程控制配置

### 跨机器部署

**机器A（RoboCap + 发送器）：**
```bash
cd rebocap
./start_udp_sender.sh <机器B的IP> 5678 7690
```

**机器B（OpenArm + 控制器）：**
```bash
cd openarm_teleop/build
./udp_openarm_control /path/to/openarm.urdf right_arm can0 5678
```

### 网络配置

1. **确保网络连通：**
   ```bash
   ping <目标机器IP>
   ```

2. **配置防火墙：**
   ```bash
   sudo ufw allow 5678/udp
   ```

3. **推荐使用有线网络** 以获得更稳定的延迟

## 故障排查

### 问题1: 收不到UDP数据

**检查端口是否监听：**
```bash
sudo netstat -ulnp | grep 5678
```

**测试UDP连接：**
```bash
# 终端1（接收）
nc -ul 5678

# 终端2（发送）
echo "test" | nc -u 127.0.0.1 5678
```

### 问题2: 防火墙阻止

```bash
# 查看防火墙状态
sudo ufw status

# 允许UDP端口
sudo ufw allow 5678/udp
```

### 问题3: 编译失败 - 找不到nlohmann/json

```bash
# 方法1: 使用包管理器
sudo apt-get install nlohmann-json3-dev

# 方法2: 手动安装
git clone https://github.com/nlohmann/json.git
cd json && mkdir build && cd build
cmake .. && sudo make install
```

### 问题4: Python找不到rebocap_ws_sdk

```bash
# 确保在正确的目录运行
cd rebocap
python3 robocap_udp_sender.py

# 或设置PYTHONPATH
export PYTHONPATH=$PYTHONPATH:/path/to/rebocap
```

### 问题5: 延迟过高

- 使用本地回环地址 `127.0.0.1` 进行本地测试
- 使用有线网络而不是WiFi
- 检查网络拥塞：`ping <目标IP>`
- 检查CPU负载：`top`

### 问题6: 关节角度不正确

需要调整关节映射函数。在 `robocap_udp_sender.py` 中修改：

```python
def map_to_openarm_joints(self, shoulder, elbow, wrist, hand):
    # 根据你的机器人调整这些映射
    shoulder_euler = self.quat_to_euler(shoulder)
    elbow_euler = self.quat_to_euler(elbow)
    wrist_euler = self.quat_to_euler(wrist)
    
    joints = [
        shoulder_euler[2],      # 可能需要调整符号或添加偏移
        shoulder_euler[1],
        shoulder_euler[0],
        elbow_euler[1],
        wrist_euler[2],
        wrist_euler[1],
        wrist_euler[0],
    ]
    return joints
```

## 参数调整

### 关节映射校准

1. 运行测试发送器观察机器人运动
2. 根据实际运动调整关节映射
3. 可能需要改变符号（+/-）或添加偏移量
4. 逐个关节测试和调整

### 夹爪控制校准

在 `robocap_udp_sender.py` 中调整：

```python
def estimate_gripper_position(self, hand_quat):
    roll, pitch, yaw = self.quat_to_euler(hand_quat)
    
    # 根据你的手势调整这个映射
    gripper = np.clip((roll + np.pi/4) / (np.pi/2), 0.0, 1.0)
    return gripper
```

### 控制参数优化

检查并调整 `config/follower.yaml` 中的控制参数：
- Kp: 位置增益
- Kd: 速度增益
- Fc, k, Fv, Fo: 摩擦补偿参数

## 代码接口

### UDP接收器API

```cpp
#include <udp_joint_receiver.hpp>

// 创建接收器（端口，关节数）
UdpJointReceiver receiver(5678, 7);

// 获取关节角度
std::vector<double> joints;
if (receiver.get_joint_angles(joints)) {
    // 有新数据可用
}

// 获取夹爪位置
double gripper = receiver.get_gripper_position();

// 获取时间戳
double timestamp = receiver.get_timestamp();

// 检查数据是否就绪
bool ready = receiver.is_data_ready();
```

## 性能优化建议

### 短期优化
1. 调整关节映射以匹配你的机器人
2. 校准夹爪控制手势
3. 优化控制参数适应UDP延迟

### 长期优化
1. 使用二进制协议替代JSON（减少包大小）
2. 实现数据压缩
3. 添加TCP选项提供可靠传输
4. 实现重连机制

## 许可证

Copyright 2025 Enactic, Inc.

Licensed under the Apache License, Version 2.0.
