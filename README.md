# 人形机器人走到目标位姿

输入起始和目标平面位姿 `(x, y, yaw)`，输出平地上的人形机器人步态轨迹。基线按「转向目的地 → 交替迈步 → 调整最终朝向」规划双脚落点，生成先转移骨盆、再抬摆动脚的轨迹，并用双连杆逆运动学计算膝关节位置。目标位姿对应末帧的骨盆水平位置和偏航角，双脚在终点恢复到并脚站姿的默认站宽。

当前模型假定地面水平且无障碍。它生成和验证运动学参考轨迹。碰撞、摩擦、动态平衡、关节限位和真机控制需要在接入具体机器人模型后验证。

## 运行

需要 Python 3.10+，只使用标准库。在仓库根目录执行：

```bash
python3 -m humanoid_walk --start 0 0 0 --goal 1.5 0.5 1.57 --csv /tmp/humanoid-walk.csv
python3 -m unittest discover -s tests -v
```

`x`、`y` 和高度用米，`yaw` 用弧度，绕世界坐标系的正 Z 轴逆时针为正。省略 `--start` 时从 `(0, 0, 0)` 出发。CSV 包含时间、骨盆和双脚位姿、双膝世界坐标，以及 `left`、`right` 或 `double` 支撑相位。腿长不可达时，CLI 会在写入轨迹前报错。

也可以从 Python 调用，并为不同腿长或采样频率传入参数：

```python
from humanoid_walk.planner import Gait, Pose
from humanoid_walk.simulation import simulate

frames = simulate(Pose(0, 0, 0), Pose(1.5, 0.5, 1.57),
                  gait=Gait(stride=0.15, step_time=0.8), hz=100)
```

## 从哪里改

| 想改什么 | 入口 |
| --- | --- |
| 起点、终点、输出路径 | CLI 的 `--start`、`--goal`、`--csv` |
| 步幅、站宽、转向角增量、抬脚高度、单步时长、腿长 | [planner.py](humanoid_walk/planner.py) 中的 `Gait`。`stride` 控制相邻直行落点的前进间距，同一只脚连续两次落地最多相隔约两个步幅 |
| 绕障碍物走、选择下一步落点 | [planner.py](humanoid_walk/planner.py) 中的 `plan()`。保持 `Step` 的左右脚、起点和落点接口，可复用轨迹生成 |
| 摆腿曲线、骨盆移动、采样频率 | [simulation.py](humanoid_walk/simulation.py) 中的 `simulate()` |
| 接入具体机型、计算关节目标 | 从 `Frame` 读取骨盆和双脚参考轨迹，将 `knee_position()` 的几何模型替换为机型对应的全身 IK，并在物理仿真中验证接触和平衡 |

## 推荐阅读顺序

1. **Kajita 等，2001，The 3D Linear Inverted Pendulum Mode: A Simple Modeling for a Biped Walking Pattern Generation。** 先理解如何用恒高质心和足底支撑简化双足动力学。读完可尝试用模型生成质心轨迹，替换当前的骨盆插值。
2. **Kajita 等，2003，Biped Walking Pattern Generation by Using Preview Control of Zero-Moment Point。** 了解如何从未来几步的支撑位置生成 ZMP 参考，再通过预瞄控制求质心轨迹。[DOI](https://doi.org/10.1109/ROBOT.2003.1241826)
3. **Pratt 等，2006，Capture Point: A Step toward Humanoid Push Recovery。** 理解如何根据质心状态调整落脚点，适合在物理仿真中加入受扰后的反馈。
4. **Kuffner 等，2001，Footstep Planning for a Humanoid Robot Among Obstacles。** 如果要从空地走向绕障碍物，重点看左右脚状态、可行落脚动作和碰撞检查。
