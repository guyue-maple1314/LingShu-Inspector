# 坐标系说明

主要服务 1.4、1.6、1.7、1.8、1.9。

| 坐标系 | 用途 | 说明 |
|---|---|---|
| `map` | 全局地图 | SLAM/BIM 对齐后的世界系 |
| `odom` | 里程计 | 短期连续定位，漂移累计 |
| `base_link` | 机器人本体 | 通常位于机身中心 |
| `lidar_link` | 激光雷达 | 相对 base_link 静态变换 |
| `imu_link` | 惯性测量单元 | 相对 base_link 静态变换 |
| `camera_left_link` | 多目左相机 | 相对 base_link 静态变换 |
| `thermal_link` | 红外相机 | 相对 base_link 静态变换 |
| `microphone_link` | 8 麦克风阵列 | 相对 base_link 静态变换 |
| `foot_fl/fr/rl/rr` | 四足足端 | 运动学输出，服务足端力约束 |

变换关系：

```text
map ──> odom ──> base_link ──> lidar_link / imu_link / camera_left_link / thermal_link / microphone_link
base_link ──> foot_fl / foot_fr / foot_rl / foot_rr
```

具体标定文件路径在 `inspection_bringup/config/devices.yaml` 中配置，待项目方提供硬件标定后冻结。
