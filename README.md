# Adaptive RBF Backstepping Controller

基于 Lyapunov 理论的自适应 RBF 神经网络反步法控制器 — C++ 类封装，PX4/AirSim 飞控仿真验证。

---

## 项目概述

从 Lyapunov 稳定性理论出发，设计自适应反步法控制器，结合 RBF 神经网络在线逼近未知动态，封装为独立 C++ 类。在 PX4 + AirSim 四旋翼仿真中完成部署与量化对比。

```
期望轨迹 → [自适应反步法] → [RBF 逼近] → 控制量 → PX4 姿态环 → 电机
```

---

## 目录结构

```
├── RBF_backstepping_Adaptive_controller/
│   ├── RBF_backstepping_Adaptive.h         # 头文件
│   ├── RBF_backstepping_Adaptive.cpp       # 核心算法实现
│   └── RBF_backstepping_Adaptive_main.cpp  # 测试主程序
│
├── PID_controller/
│   └── PID_control.c                       # C 语言 PID 跟踪正弦
│
├── test_example/
│   ├── backstepping.c                      # C 语言反步法
│   ├── backstepping.cpp                    # C++ 反步法
│   └── Calculator.cpp                      # C++ 类入门
│
└── README.md
```

---

## 核心技术

| 技术 | 说明 |
|------|------|
| 反步法 | 递推构造 Lyapunov 函数，两步推导控制律 |
| RBF 神经网络 | 高斯基函数加权求和，在线逼近未知非线性 |
| 自适应律 | `Ẇ=γ·τ·Φ(x)-σ·W`，从 Lyapunov 求导导出 |
| σ-修正 | 防权重漂移，保证闭环信号有界 |
| 鲁棒补偿 | 压住神经网络逼近残差 |
| 变化率限制 | 加速度平滑过渡，消除姿态抖动 |

---

## 仿真验证

PX4 + AirSim 四旋翼，正弦跟踪 5·sin(0.5t)：

| 测试场景 | 自适应反步法 RMS | PX4 默认 PID RMS |
|----------|:--------------:|:--------------:|
| 无风 | **0.22 m** | 1.95 m |
| 3m/s 侧风 | **0.21 m** | 2.34 m |

自适应控制器对恒定风扰几乎免疫，RBF 权重自动学习风扰补偿。

---

## 编译运行

```bash
# C++ 测试
g++ RBF_backstepping_Adaptive_main.cpp RBF_backstepping_Adaptive.cpp -o RBF_backstepping_Adaptive_test.exe
./RBF_backstepping_Adaptive_test.exe
```

```bash
# Python（AirSim 仿真）
pip install mavsdk
cd mavsdk
cp /mnt/d/文件夹/test_offboard_controller.py ./
python3 test_offboard_controller.py
```

---

## 参考

- Khalil, H. K. *Nonlinear Systems*, 3rd Ed.
- Krstić, M. et al. *Nonlinear and Adaptive Control Design*. Wiley, 1995.
- Lai, G. et al. Adaptive Position/Attitude Tracking Control of Aerial Robot. *IEEE TNNLS*, 2016.
