# RealSense机械臂系统

这个模块提供了完整的RealSense相机与机械臂集成解决方案，包括目标检测、位姿获取、相机标定和坐标系转换功能。

## 功能特性

- **RealSense相机控制**: 初始化、配置和控制RealSense相机
- **目标位姿检测**: 支持ArUco标记和颜色目标检测
- **相机标定**: 相机内参标定和手眼标定
- **坐标系转换**: 建立相机坐标系与机器人基座标系的关系
- **T-pose标定**: 支持人体姿态标定
- **实时检测**: 实时目标位姿检测和可视化

## 文件结构

```
RealSense/
├── realsense_camera.py      # RealSense相机控制模块
├── pose_detection.py        # 位姿检测模块
├── camera_calibration.py    # 相机标定模块
├── coordinate_system.py     # 坐标系标定模块
├── main_example.py          # 主程序示例
└── README.md               # 使用说明
```

## 依赖安装

```bash
pip install pyrealsense2
pip install opencv-python
pip install numpy
pip install json
```

## 快速开始

### 1. 运行主程序

```bash
python main_example.py
```

### 2. 系统初始化

程序启动后会自动初始化RealSense相机和各个功能模块。

### 3. 相机标定

1. 准备9x6的棋盘格标定板
2. 在主菜单选择"1. 相机标定"
3. 按照提示移动标定板捕获15张不同角度的图像
4. 系统会自动计算相机内参并保存结果

### 4. 坐标系标定

1. 准备ArUco标记或彩色目标物
2. 在主菜单选择"2. 坐标系标定"
3. 按照提示在机器人基座标系的关键点放置目标物
4. 系统会建立相机坐标系与机器人基座标系的变换关系

### 5. 实时位姿检测

1. 在主菜单选择"3. 实时位姿检测"
2. 系统会显示实时图像和检测结果
3. 按空格键保存当前检测结果
4. 按ESC键退出

## 模块使用说明

### RealSenseCamera类

```python
from realsense_camera import RealSenseCamera

# 创建相机对象
camera = RealSenseCamera()

# 启动数据流
camera.start_streaming()

# 获取图像
color_image, depth_image = camera.get_frames()

# 停止数据流
camera.stop_streaming()
```

### PoseDetector类

```python
from pose_detection import PoseDetector

# 创建检测器
detector = PoseDetector(camera)

# 检测目标
detections = detector.detect_targets(color_image, depth_image)

# 处理检测结果
for detection in detections:
    position = detection['position']  # 3D位置
    bbox = detection['bbox']          # 边界框
    confidence = detection['confidence']  # 置信度
```

### CameraCalibrator类

```python
from camera_calibration import CameraCalibrator

# 创建标定器
calibrator = CameraCalibrator(camera)

# 捕获标定图像
calibrator.capture_calibration_images(num_images=15)

# 执行标定
calibrator.calibrate_camera()

# 保存结果
calibrator.save_calibration_result()
```

### CoordinateSystemCalibrator类

```python
from coordinate_system import CoordinateSystemCalibrator

# 创建坐标系标定器
coord_calibrator = CoordinateSystemCalibrator(camera, pose_detector)

# 添加标定点
coord_calibrator.add_calibration_point(camera_point, robot_point, "point_name")

# 执行标定
coord_calibrator.calibrate_coordinate_system()

# 转换坐标
robot_points = coord_calibrator.transform_points(camera_points)
```

## 标定流程

### 相机内参标定

1. 打印9x6的棋盘格标定板（方格大小2.5cm）
2. 运行标定程序，移动标定板到不同位置和角度
3. 捕获15-20张清晰的标定图像
4. 系统自动计算相机内参和畸变系数

### 坐标系标定

1. 在机器人基座标系中选择4-5个已知坐标的点
2. 将目标物（ArUco标记或彩色物体）放置在这些点上
3. 使用相机检测目标物的相机坐标
4. 建立相机坐标与机器人坐标的对应关系
5. 计算变换矩阵

### 手眼标定（可选）

1. 将相机固定在机器人末端
2. 在工作空间中放置标定目标
3. 移动机器人到不同位姿观察目标
4. 记录机器人位姿和相机观测结果
5. 计算手眼变换矩阵

## 配置参数

### 相机参数

- 分辨率: 640x480 (可在realsense_camera.py中修改)
- 帧率: 30fps
- 深度范围: 0.1-10米

### 检测参数

- ArUco字典: DICT_6X6_250
- 颜色检测HSV范围: 可在pose_detection.py中调整
- 最小目标尺寸: 可配置

### 标定参数

- 棋盘格尺寸: 9x6内角点
- 方格大小: 2.5cm
- 最小标定图像数: 10张

## 注意事项

1. **光照条件**: 确保工作环境有充足且均匀的光照
2. **标定板质量**: 使用高质量打印的棋盘格标定板
3. **目标物选择**: ArUco标记检测更稳定，彩色目标需要良好的颜色对比
4. **标定精度**: 标定点分布要均匀，覆盖整个工作空间
5. **坐标系一致性**: 确保机器人坐标系定义与实际一致

## 故障排除

### 相机无法启动
- 检查RealSense驱动是否正确安装
- 确认相机USB连接正常
- 检查是否有其他程序占用相机

### 检测不到目标
- 检查光照条件
- 调整检测参数
- 确认目标物在相机视野内

### 标定精度差
- 增加标定点数量
- 确保标定点分布均匀
- 检查标定板或目标物的质量

### 坐标转换错误
- 重新进行坐标系标定
- 检查机器人坐标系定义
- 验证标定点的准确性

## 扩展功能

可以根据需要扩展以下功能：

1. **多相机支持**: 支持多个RealSense相机
2. **深度学习检测**: 集成YOLO等深度学习目标检测
3. **轨迹规划**: 基于检测结果进行路径规划
4. **实时控制**: 与机器人控制系统实时通信
5. **数据记录**: 记录和回放检测数据

## 联系方式

如有问题或建议，请联系开发团队。