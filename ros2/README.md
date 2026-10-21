# ROS 2 camera bridge

Stonefish 负责仿真并每 10 Hz 写出 `images/ego/*.png` 和 `images/wrist/*.png`；
本目录的 `camera_publisher.py` 用 ROS 2 把它们发布成 `sensor_msgs/Image`，
RViz2 直接订阅即可。

## 先启动仿真

```bash
cd /media/ai/DATA1/dcw/stonefish/uuv_charge_nav
export LD_LIBRARY_PATH=/media/ai/DATA1/dcw/stonefish/install_stonefish/lib:$LD_LIBRARY_PATH
mkdir -p episodes_raw/live_demo
cp examples/demo_episode/* episodes_raw/live_demo/
DISPLAY=:1 ./build/uuv_nav \
  --scene "$PWD/episodes_raw/live_demo/scene.scn" \
  --episode-dir "$PWD/episodes_raw/live_demo" \
  --show-camera
```

> `--show-camera` 可选；它只是把 ego 画面叠加到 Stonefish 主窗口。
> ROS 2 发布读取的是 `--episode-dir` 下记录的 PNG，不依赖这个开关。

## 再启动 ROS 2 发布 + RViz

```bash
cd /media/ai/DATA1/dcw/stonefish/uuv_charge_nav
./ros2/run_camera_view.sh "$PWD/episodes_raw/live_demo"
```

脚本会：

1. `source /opt/ros/humble/setup.bash`
2. 用系统 Python 3.10 启动 `ros2/camera_publisher.py`
3. 启动 `rviz2 -d ros2/uuv_cameras.rviz`

RViz 默认显示 **`/bluerov2/cameras/combined`**：左半幅 ego、右半幅 wrist 的
640×240 拼接图，一个 Image display 就能同时看到两个相机。

## Topics

| Topic | 内容 |
|---|---|
| `/bluerov2/camera_ego/image_raw` | 前视右相机，640×480 RGB8 |
| `/bluerov2/camera_wrist/image_raw` | 前视左/ hand 相机，640×480 RGB8 |
| `/bluerov2/camera_ego/camera_info` | 100° HFOV 内参 |
| `/bluerov2/camera_wrist/camera_info` | 100° HFOV 内参 |
| `/bluerov2/cameras/combined` | ego \| wrist 拼接图，640×240 RGB8 |

发布频率约 10 Hz（每路相机）；combined 也是 10 Hz。

## 单独查看某一路

```bash
source /opt/ros/humble/setup.bash

# 命令行确认主题
ros2 topic list | grep bluerov2
ros2 topic hz /bluerov2/camera_ego/image_raw

# 用 RViz 的 Image display 订阅 /bluerov2/camera_ego/image_raw
rviz2
```

也可以直接用 `rqt_image_view` / `image_view`：

```bash
ros2 run rqt_image_view rqt_image_view
```

## 注意

- 该节点必须用 **系统 Python 3.10**（`/usr/bin/python3`），不要用项目的
  `../.venv`（Python 3.11）；ROS 2 Humble 的 `rclpy` 是 3.10 编译的。
- 发布节点只负责“把磁盘上最新 PNG 发出去”；仿真停止后它会停留在最后一帧。
- 我们在 RViz Humble 中实测：同时启用两个 `rviz_default_plugins/Image` display
  可能触发渲染窗口崩溃，因此默认 RViz 配置使用一个 combined Image display。
  如果你确实要两个独立窗口，可以开两个 RViz 实例，各放一个 Image display，
  分别订阅上面两个 `image_raw` 主题。
- 相机图像是仿真记录帧（10 Hz），不是 30 Hz 的实时渲染流；这与数据集采样率一致。
