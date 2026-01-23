# OpenArm OriginFlow

## Project Intro

- **openarm_mujoco**

  mujoco 仿真配置文件

- **openarm_teleop**(C++)

  遥操通讯(udp接受端)及控制代码

- **openarm_description**(ROS)

  urdf 完整配置文件以及各部分 xacro 配置文件

- **openarm_test**(ROS)

  机械臂测试功能包, 可进行轨迹发布

- **rebocap_udp_sender**(Python)

  rebocap数据接收以及处理, udp发送端


## Dependencies

1. ROS2 Humble(自行安装, 只使用遥操功能的话可以不用安装ROS2)

2. OpenArm CAN 通讯库

    ```shell
    sudo apt install -y software-properties-common
    sudo add-apt-repository -y ppa:openarm/main
    sudo apt update
    sudo apt install -y \
      can-utils \
      iproute2 \
      libopenarm-can-dev \
      openarm-can-utils
    ```
3. OpenArm Teleop 

    ```shell
    sudo apt update
    sudo apt install -y \
      libeigen3-dev \
      libopenarm-can-dev \
      liborocos-kdl-dev \
      liburdfdom-dev \
      liburdfdom-headers-dev \
      libyaml-cpp-dev \
    ```

## Usage

### 0. 通讯配置

完整机械臂达妙电机 ID 已配置完毕, 具体 Sender ID 和 Receiver ID 参考以下链接

https://docs.openarm.dev/software/setup/motor-id

用户需设置 CAN Interface, 首先将 CAN 接口插入电脑 USB 口, 然后使用以下命令查看识别到的 CAN Interface ID

```shell
ip link show
```

然后使用以下命令配置对应的 CAN Interface ID, 注意 <can_id> 要替换为电脑实际识别到的 CAN Interface ID, 一般为 can0(右臂) 或 can1(左臂)

```shell
openarm-can-configure-socketcan <can_id> -fd -b 1000000 -d 5000000
```

### 1. ROS功能包编译

到当前仓库路径下运行以下命令

```shell
sudo chmod +x build.sh
./build.sh
source install/setup.bash
```

### 2. 发送端设置(遥操)

发送端是纯粹的 python 语法, 核心文件为 rebocap_udp_sender 下的 rebocap_udp_sender.py, 主要包括 ReboCap 数据接收功能以及关节映射功能(developing)

在 Windows 系统上启动 ReboCap 并确定可接收到关节数据后, 使用以下命令启动发送端

```shell
python rebocap_udp_sender.py <udp_host> <udp_port> <rebocap_port>
```

三个参数均为可选参数, udp_host 默认为 255.255.255.255(广播地址), upd_port 默认为 5678, rebocap_port 默认为 7690

### 3. 接受端设置(遥操)

接受端是纯粹的 C++ 语法, 核心文件为 openarm_teleop/control 下的 udp_openarm_control.cpp, 使用以下命令完成对接收端代码的编译

```shell
cd openarm_teleop
mkdir build && cd build
cmake ..
make -j4
```

在 openarm_teleop 文件夹下使用以下命令

```shell
./build/robocap_control <urdf_path> <arm_side> <can_interface> <udp_port>
```
urdf_path 为必选参数, 其他为可选, urdf 文件在该仓库下的 openarm_description/urdf/openarm.urdf 路径下, 传入参数时请使用绝对路径

arm_side 默认为 left_arm, can_interface 默认为 can1， udp_port 默认为 5678

### 4. 机械臂简单测试

openarm_ros2 仓库和 openarm_test 仓库用于机械臂测试, 目前可复现 npz 存储的关节轨迹(支持全关节或上臂三关节), 支持仿真复现和实机复现

使用以下命令启动实机侧

```shell
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

没有实机可启动仿真侧

```shell
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=True
```

然后启动轨迹发送端, 可修改 openarm_test/config 中的 openarm_test.yaml 文件参数来指定要读取的数据, 所有数据文件默认放在 openarm_test/data 路径下

```shell
ros2 launch openarm_test openarm_test.launch.py
```
