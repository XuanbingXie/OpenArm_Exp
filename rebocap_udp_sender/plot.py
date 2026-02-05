import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

# 配置
FILENAME = 'debug_joints.csv'
TOTAL_JOINTS = 14

def main():
    if not os.path.exists(FILENAME):
        print(f"错误: 找不到文件 {FILENAME}")
        print("请确保先运行主程序生成 CSV 数据文件。")
        return

    try:
        df = pd.read_csv(FILENAME)
    except Exception as e:
        print(f"读取 CSV 失败: {e}")
        return

    required_cols = ['timestamp'] + [f'pre_j{i+1}' for i in range(TOTAL_JOINTS)] + [f'post_j{i+1}' for i in range(TOTAL_JOINTS)]
    if not all(col in df.columns for col in required_cols):
        print("CSV 文件格式不符合预期 (缺少部分列)")
        return

    # 初始化绘图
    # sharex=True 实现了上下两图 X 轴联动 (缩放/平移同步)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    plt.subplots_adjust(bottom=0.2) # 留出底部空间用于显示说明

    # 状态变量：当前显示的关节索引 (0 代表 Joint 1)
    state = {'current_joint_idx': 0}

    def update_plot():
        """根据当前关节索引重新绘制图表"""
        idx = state['current_joint_idx']
        joint_num = idx + 1
        
        pre_col = f'pre_j{joint_num}'
        post_col = f'post_j{joint_num}'

        # 清除旧图
        ax1.cla()
        ax2.cla()

        # 获取数据
        timestamps = df['timestamp']
        pre_data = df[pre_col]
        post_data = df[post_col]

        # 绘制 Pre (上方)
        # linestyle='None' 确保不连线，marker='.' 显示为点
        ax1.plot(timestamps, pre_data, color='blue', marker='.', linestyle='None', alpha=0.7, label=f'Pre Joint {joint_num}')
        ax1.set_ylabel('Angle (rad/deg)')
        ax1.set_title(f'Joint {joint_num} - Pre-Interpolation (Raw)')
        ax1.grid(True, linestyle='--', alpha=0.5)
        ax1.legend(loc='upper right')

        # 绘制 Post (下方)
        ax2.plot(timestamps, post_data, color='red', marker='.', linestyle='None', alpha=0.7, label=f'Post Joint {joint_num}')
        ax2.set_xlabel('Timestamp')
        ax2.set_ylabel('Angle (rad/deg)')
        ax2.set_title(f'Joint {joint_num} - Post-Interpolation (Smoothed)')
        ax2.grid(True, linestyle='--', alpha=0.5)
        ax2.legend(loc='upper right')

        # 刷新画布
        fig.canvas.draw_idle()
        print(f"Switched to Joint {joint_num}")

    def on_key_press(event):
        """键盘事件处理"""
        # 数字键 0-9 快速切换 Joint 1-10
        if event.key in [str(i) for i in range(10)]:
            # 逻辑：按下 '1' 对应 Index 0 (Joint 1), 按下 '0' 对应 Index 9 (Joint 10)
            key_num = int(event.key)
            if key_num == 0:
                target_idx = 9 
            else:
                target_idx = key_num - 1
            
            if target_idx < TOTAL_JOINTS:
                state['current_joint_idx'] = target_idx
                update_plot()
        
        # 左右方向键循环切换 (可访问所有 14 个关节)
        elif event.key == 'right':
            state['current_joint_idx'] = (state['current_joint_idx'] + 1) % TOTAL_JOINTS
            update_plot()
        elif event.key == 'left':
            state['current_joint_idx'] = (state['current_joint_idx'] - 1) % TOTAL_JOINTS
            update_plot()

    # 绑定事件
    fig.canvas.mpl_connect('key_press_event', on_key_press)

    # 首次绘制
    update_plot()

    # 添加操作说明文本
    info_text = (
        "Controls:\n"
        "Key [1-9]: Select Joint 1-9  |  Key [0]: Select Joint 10\n"
        "Key [Left/Right]: Cycle through ALL 14 Joints\n"
        "Mouse: Use toolbar buttons to Pan/Zoom (affects both plots)"
    )
    plt.figtext(0.5, 0.02, info_text, ha="center", fontsize=10, bbox={"facecolor":"orange", "alpha":0.2, "pad":5})

    print("Plotter started. Use Left/Right keys to switch joints.")
    plt.show()

if __name__ == "__main__":
    main()