# 参数说明

本文档说明 `config/planner.yaml` 中的可调参数。YAML 文件只保留参数值，详细含义、单位和可选项统一放在这里。

## 基本约定

- 长度、位置、分辨率单位为 m。
- 时间单位为 s。
- 参数名以 `_deg` 结尾的角度单位为 degree，其余角度参数单位为 rad。
- 当前版本只保留双向 Ackermann：前端会采样前进和后退 Ackermann primitive。
- 碰撞检测和安全走廊使用相同的双圆模型。单个圆半径为 `hypot(robot_length / 4, robot_width / 2) + collision_clearance`，前后圆心相对车体中心的距离为 `robot_length / 4`。

## Demo Map

| 参数 | 含义 | 可选项 |
| --- | --- | --- |
| `map_scenario` | 内置地图场景。RViz 参数面板可在线切换。 | `legacy_maze`、`legacy_maze_inflated_0_1`、`legacy_parking`、`reference_parking`、`tight_complex` |
| `resolution` | 内置地图发布的 OccupancyGrid 分辨率。 | 正数 |
| `frame_id` | `/map` 和可视化 Marker 使用的坐标系。 | 通常为 `map` |

## Input & Occupancy

| 参数 | 含义 | 可选项 |
| --- | --- | --- |
| `obstacle_threshold` | OccupancyGrid 中大于等于该值的栅格视为障碍。 | `0` 到 `100` |
| `unknown_is_obstacle` | 是否把未知栅格 `-1` 当成障碍。 | `true` 更保守；`false` 更开放 |

## Search Grid

| 参数 | 含义 | 调参影响 |
| --- | --- | --- |
| `xy_resolution` | 前端闭集哈希的 x/y 离散分辨率。 | 越小越精细，节点数越多 |
| `yaw_resolution` | 前端闭集哈希的 yaw 离散分辨率。 | 越小方向区分越细，搜索更慢 |
| `heuristic_resolution` | 障碍感知 2D Dijkstra 启发函数的栅格分辨率。 | 越小启发更细，预计算更慢 |
| `max_iterations` | 前端 Hybrid A* 最大搜索迭代次数。 | 防止搜索无限扩展 |

## Goal & Analytic

| 参数 | 含义 | 可选项或影响 |
| --- | --- | --- |
| `goal_distance_tolerance` | 终点位置误差容许范围。 | 越小越严格 |
| `goal_yaw_tolerance` | 终点航向误差容许范围。 | 越小越严格 |
| `analytic_expansion_distance` | 启发距离小于该值时尝试 OMPL Reeds-Shepp 终端解析连接。 | 设为 `0.0` 可基本关闭主动终端连接 |

## Vehicle Geometry

| 参数 | 含义 |
| --- | --- |
| `wheelbase` | Ackermann 等效轴距。 |
| `robot_length` | 车体长度，用于可视化、碰撞双圆和安全走廊前后圆心。 |
| `robot_width` | 车体宽度，用于可视化、碰撞双圆和安全走廊半径。 |
| `collision_clearance` | 额外安全膨胀半径，同时作用于前端碰撞检测和后端安全走廊。 |

## Vehicle Limits

| 参数 | 含义 | 调参影响 |
| --- | --- | --- |
| `reference_velocity` | 前端 primitive rollout 使用的参考速度。 | 影响 primitive 长度 |
| `max_steer_deg` | Ackermann 最大转角。 | 影响最小转弯半径 |

## Motion Sampling

| 参数 | 含义 | 调参影响 |
| --- | --- | --- |
| `sampling_time` | 单个 motion primitive 的持续时间。 | 越大步长越长，越小搜索更细 |
| `integration_dt` | rollout primitive 的内部积分步长。 | 越小碰撞检测和轨迹积分更细 |

## Front-End Costs

| 参数 | 含义 |
| --- | --- |
| `reverse_penalty` | 倒车距离代价倍率。 |
| `steer_penalty` | 转角绝对值代价系数。 |
| `steer_change_penalty` | 相邻 primitive 转角变化代价系数。 |
| `direction_change_penalty` | 前进/后退切换代价。 |

## Backend Solver

| 参数 | 含义 | 可选项或影响 |
| --- | --- | --- |
| `backend_casadi_python` | 带 CasADi 环境的 Python 解释器路径。 | 例如 Isaac Sim 或 conda 环境下的 `python3` |
| `backend_casadi_script` | CasADi 后端脚本路径。 | 默认使用包内 `scripts/casadi_backend.py` |
| `backend_max_iterations` | 后端外层最大迭代次数。每轮都会围绕当前轨迹重建安全走廊并调用一次 CasADi/IPOPT。 | 论文中的后端迭代上限；`1` 表示只做一次后端优化 |
| `backend_ipopt_max_iterations` | 单次 IPOPT NLP 求解的内部最大迭代次数。 | 越大更可能收敛，但耗时增加 |
| `backend_ipopt_tol` | IPOPT 一阶最优性容差。 | 越小越严格 |

后端固定为 Python CasADi + IPOPT：前端规划成功后会自动进入后端优化，不再保留旧 C++ IPOPT 备用后端或后端启停开关。

## Backend Corridor

| 参数 | 含义 | 调参影响 |
| --- | --- | --- |
| `backend_resample_distance` | 优化前按累计弧长重新均匀采样的目标间距。 | 越小 NLP 点数越多，轨迹更细但更慢 |
| `backend_corridor_max_distance` | 单个安全走廊每个方向最大扩展距离。 | 越大可行空间更宽，但构建和可视化更重 |
| `backend_corridor_fast_step` | 安全走廊快速轮询扩展步长。 | 用于快速接近障碍或边界 |
| `backend_corridor_fine_step` | 快速扩展停止后回退区间内的精细扩展步长。 | 越小边界更准确 |
| `backend_corridor_axis_aligned` | 安全走廊坐标系。 | `true`：世界坐标轴对齐；`false`：速度方向局部坐标对齐 |

安全走廊保留快慢分步轮询扩展，并分别基于车体前后双圆圆心建立走廊约束。

## Backend Objective & Bounds

| 参数 | 含义 |
| --- | --- |
| `backend_comfort_weight` | 舒适度权重。CasADi 后端目标包含 `T + alpha * sum((v * omega)^2) * dt`。 |
| `backend_constraint_penalty` | 软约束惩罚权重。 |
| `backend_infeasibility_tolerance` | 后端可接受不可行度/外层收敛阈值。 |
| `backend_max_velocity` | 后端优化变量 `v_k` 的速度上界。 |
| `backend_max_acceleration` | 后端优化变量 `a_k` 的加速度上界。 |

## Visualization

| 参数 | 含义 |
| --- | --- |
| `pose_arrow_length` | RViz 起点/终点箭头长度。 |
| `pose_arrow_width` | RViz 起点/终点箭头宽度。 |
| `pose_arrow_height` | RViz 起点/终点箭头高度。 |
| `pose_marker_z` | 起点/终点箭头的 z 偏移。 |
| `body_trajectory_visualization_enabled` | 是否显示最终轨迹上的单个可动车体框。只播放最终轨迹，不播放前端轨迹。 |
| `body_trajectory_marker_z` | 可动车体框的 z 偏移。 |
| `backend_corridor_visualization_enabled` | 是否显示后端安全走廊线框。 |
| `backend_corridor_visualization_stride` | 每隔多少个后端重采样点显示一个安全走廊。 |
| `backend_corridor_marker_z` | 安全走廊线框的 z 偏移。 |
