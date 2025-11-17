#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
坐标系确定模块
用于建立机械臂基座标系和相机坐标系的关系
"""

import cv2
import numpy as np
import json
import os
from typing import List, Tuple, Optional, Dict, Any
from realsense_camera import RealSenseCamera
from pose_detection import PoseDetector
from camera_calibration import CameraCalibrator
import time

class CoordinateSystemCalibrator:
    """坐标系标定器"""
    
    def __init__(self, camera: RealSenseCamera, pose_detector: PoseDetector):
        """
        初始化坐标系标定器
        
        Args:
            camera: RealSense相机对象
            pose_detector: 位姿检测器
        """
        self.camera = camera
        self.pose_detector = pose_detector
        
        # 标定数据
        self.calibration_points = []  # 标定点数据
        self.robot_base_transform = None  # 相机到机器人基座的变换矩阵
        
        # T-pose相关
        self.t_pose_positions = []  # T-pose各关节位置
        self.t_pose_transform = None  # T-pose变换矩阵
        
        print("坐标系标定器初始化完成")
    
    def add_calibration_point(self, 
                            camera_point: np.ndarray, 
                            robot_point: np.ndarray,
                            point_name: str = ""):
        """
        添加标定点
        
        Args:
            camera_point: 相机坐标系中的点 [x, y, z]
            robot_point: 机器人基座标系中的点 [x, y, z]
            point_name: 点的名称
        """
        self.calibration_points.append({
            'camera_point': camera_point.copy(),
            'robot_point': robot_point.copy(),
            'name': point_name,
            'timestamp': time.time()
        })
        
        print(f"添加标定点 '{point_name}': 相机坐标{camera_point}, 机器人坐标{robot_point}")
        print(f"当前标定点数量: {len(self.calibration_points)}")
    
    def capture_calibration_point_interactive(self, point_name: str = "") -> bool:
        """
        交互式捕获标定点
        
        Args:
            point_name: 点的名称
            
        Returns:
            bool: 是否成功捕获
        """
        print(f"准备捕获标定点: {point_name}")
        print("请将标记物放置在指定位置，按空格键捕获，按ESC退出")
        
        while True:
            # 获取图像
            color_image, depth_image = self.camera.get_frames()
            
            if color_image is None:
                continue
            
            # 检测标记物
            detections = self.pose_detector.detect_targets(color_image, depth_image)
            
            # 显示图像
            display_image = color_image.copy()
            
            if detections:
                # 绘制检测结果
                for detection in detections:
                    # 绘制边界框
                    bbox = detection.get('bbox')
                    if bbox:
                        x, y, w, h = bbox
                        cv2.rectangle(display_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    
                    # 显示3D坐标
                    position = detection.get('position')
                    if position is not None:
                        pos_text = f"({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
                        cv2.putText(display_image, pos_text, (x, y-10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                status_text = f"检测到标记物! 按空格键捕获点: {point_name}"
                cv2.putText(display_image, status_text, (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                status_text = f"未检测到标记物 - {point_name}"
                cv2.putText(display_image, status_text, (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow('Coordinate Calibration', display_image)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' ') and detections:  # 空格键捕获
                # 使用第一个检测到的目标
                detection = detections[0]
                camera_point = detection['position']
                
                # 获取机器人坐标（这里需要从机器人控制系统获取）
                print(f"检测到相机坐标: {camera_point}")
                print("请输入对应的机器人基座标系坐标:")
                
                try:
                    x = float(input("X坐标 (米): "))
                    y = float(input("Y坐标 (米): "))
                    z = float(input("Z坐标 (米): "))
                    robot_point = np.array([x, y, z])
                    
                    self.add_calibration_point(camera_point, robot_point, point_name)
                    cv2.destroyAllWindows()
                    return True
                    
                except ValueError:
                    print("输入格式错误，请重新捕获")
                    continue
            
            elif key == 27:  # ESC退出
                break
        
        cv2.destroyAllWindows()
        return False
    
    def calibrate_coordinate_system(self, method: str = 'least_squares') -> bool:
        """
        标定坐标系变换
        
        Args:
            method: 标定方法 ('least_squares', 'svd', 'ransac')
            
        Returns:
            bool: 标定是否成功
        """
        if len(self.calibration_points) < 3:
            print("标定点数量不足，至少需要3个点")
            return False
        
        print(f"开始坐标系标定，使用{method}方法...")
        
        # 提取点坐标
        camera_points = np.array([point['camera_point'] for point in self.calibration_points])
        robot_points = np.array([point['robot_point'] for point in self.calibration_points])
        
        if method == 'least_squares':
            success = self._calibrate_least_squares(camera_points, robot_points)
        elif method == 'svd':
            success = self._calibrate_svd(camera_points, robot_points)
        elif method == 'ransac':
            success = self._calibrate_ransac(camera_points, robot_points)
        else:
            print(f"不支持的标定方法: {method}")
            return False
        
        if success:
            print("坐标系标定成功!")
            print(f"变换矩阵:\n{self.robot_base_transform}")
            
            # 计算标定误差
            self._calculate_calibration_error()
            
            return True
        else:
            print("坐标系标定失败")
            return False
    
    def _calibrate_least_squares(self, camera_points: np.ndarray, robot_points: np.ndarray) -> bool:
        """
        最小二乘法标定
        
        Args:
            camera_points: 相机坐标点
            robot_points: 机器人坐标点
            
        Returns:
            bool: 是否成功
        """
        try:
            # 计算质心
            camera_centroid = np.mean(camera_points, axis=0)
            robot_centroid = np.mean(robot_points, axis=0)
            
            # 去质心
            camera_centered = camera_points - camera_centroid
            robot_centered = robot_points - robot_centroid
            
            # 计算协方差矩阵
            H = camera_centered.T @ robot_centered
            
            # SVD分解
            U, S, Vt = np.linalg.svd(H)
            
            # 计算旋转矩阵
            R = Vt.T @ U.T
            
            # 确保旋转矩阵的行列式为正
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            
            # 计算平移向量
            t = robot_centroid - R @ camera_centroid
            
            # 构建变换矩阵
            self.robot_base_transform = np.eye(4)
            self.robot_base_transform[:3, :3] = R
            self.robot_base_transform[:3, 3] = t
            
            return True
            
        except Exception as e:
            print(f"最小二乘法标定失败: {e}")
            return False
    
    def _calibrate_svd(self, camera_points: np.ndarray, robot_points: np.ndarray) -> bool:
        """
        SVD方法标定
        
        Args:
            camera_points: 相机坐标点
            robot_points: 机器人坐标点
            
        Returns:
            bool: 是否成功
        """
        try:
            # 使用Kabsch算法
            # 添加齐次坐标
            camera_homo = np.hstack([camera_points, np.ones((len(camera_points), 1))])
            
            # 构建线性方程组 A * x = b
            # 其中x是变换矩阵的展开形式
            A = np.kron(camera_homo, np.eye(3))
            b = robot_points.flatten()
            
            # 求解
            x = np.linalg.lstsq(A, b, rcond=None)[0]
            
            # 重构变换矩阵
            transform_3x4 = x.reshape(3, 4)
            self.robot_base_transform = np.vstack([transform_3x4, [0, 0, 0, 1]])
            
            return True
            
        except Exception as e:
            print(f"SVD方法标定失败: {e}")
            return False
    
    def _calibrate_ransac(self, camera_points: np.ndarray, robot_points: np.ndarray) -> bool:
        """
        RANSAC方法标定
        
        Args:
            camera_points: 相机坐标点
            robot_points: 机器人坐标点
            
        Returns:
            bool: 是否成功
        """
        try:
            max_iterations = 1000
            threshold = 0.01  # 1cm
            best_inliers = 0
            best_transform = None
            
            for _ in range(max_iterations):
                # 随机选择3个点
                if len(camera_points) < 3:
                    break
                
                indices = np.random.choice(len(camera_points), 3, replace=False)
                sample_camera = camera_points[indices]
                sample_robot = robot_points[indices]
                
                # 使用这3个点计算变换
                if self._calibrate_least_squares(sample_camera, sample_robot):
                    # 计算所有点的误差
                    transformed_points = self.transform_points(camera_points)
                    errors = np.linalg.norm(transformed_points - robot_points, axis=1)
                    
                    # 统计内点
                    inliers = np.sum(errors < threshold)
                    
                    if inliers > best_inliers:
                        best_inliers = inliers
                        best_transform = self.robot_base_transform.copy()
            
            if best_transform is not None:
                self.robot_base_transform = best_transform
                print(f"RANSAC找到{best_inliers}/{len(camera_points)}个内点")
                return True
            else:
                return False
                
        except Exception as e:
            print(f"RANSAC方法标定失败: {e}")
            return False
    
    def _calculate_calibration_error(self):
        """计算标定误差"""
        if self.robot_base_transform is None:
            return
        
        camera_points = np.array([point['camera_point'] for point in self.calibration_points])
        robot_points = np.array([point['robot_point'] for point in self.calibration_points])
        
        # 变换相机坐标到机器人坐标
        transformed_points = self.transform_points(camera_points)
        
        # 计算误差
        errors = np.linalg.norm(transformed_points - robot_points, axis=1)
        
        mean_error = np.mean(errors)
        max_error = np.max(errors)
        std_error = np.std(errors)
        
        print(f"标定误差统计:")
        print(f"  平均误差: {mean_error:.4f} 米")
        print(f"  最大误差: {max_error:.4f} 米")
        print(f"  标准差: {std_error:.4f} 米")
        
        # 显示每个点的误差
        for i, (point, error) in enumerate(zip(self.calibration_points, errors)):
            print(f"  点{i+1} ({point['name']}): {error:.4f} 米")
    
    def transform_points(self, camera_points: np.ndarray) -> np.ndarray:
        """
        将相机坐标点变换到机器人基座标系
        
        Args:
            camera_points: 相机坐标点 (N, 3)
            
        Returns:
            np.ndarray: 机器人基座标系坐标点 (N, 3)
        """
        if self.robot_base_transform is None:
            raise ValueError("坐标系变换矩阵未标定")
        
        # 添加齐次坐标
        camera_homo = np.hstack([camera_points, np.ones((len(camera_points), 1))])
        
        # 应用变换
        robot_homo = (self.robot_base_transform @ camera_homo.T).T
        
        # 返回3D坐标
        return robot_homo[:, :3]
    
    def transform_pose(self, camera_pose: np.ndarray) -> np.ndarray:
        """
        将相机坐标系中的位姿变换到机器人基座标系
        
        Args:
            camera_pose: 相机坐标系中的位姿 (4x4矩阵)
            
        Returns:
            np.ndarray: 机器人基座标系中的位姿 (4x4矩阵)
        """
        if self.robot_base_transform is None:
            raise ValueError("坐标系变换矩阵未标定")
        
        return self.robot_base_transform @ camera_pose
    
    def calibrate_t_pose(self, joint_positions: Dict[str, np.ndarray]) -> bool:
        """
        标定T-pose
        
        Args:
            joint_positions: 关节位置字典 {'joint_name': [x, y, z]}
            
        Returns:
            bool: 是否成功
        """
        print("开始T-pose标定...")
        
        # 保存T-pose位置
        self.t_pose_positions = joint_positions.copy()
        
        # 计算T-pose的特征变换（例如肩膀连线作为X轴）
        if 'left_shoulder' in joint_positions and 'right_shoulder' in joint_positions:
            left_shoulder = joint_positions['left_shoulder']
            right_shoulder = joint_positions['right_shoulder']
            
            # 肩膀连线作为X轴
            x_axis = right_shoulder - left_shoulder
            x_axis = x_axis / np.linalg.norm(x_axis)
            
            # Z轴向上（假设）
            z_axis = np.array([0, 0, 1])
            
            # Y轴通过叉积计算
            y_axis = np.cross(z_axis, x_axis)
            y_axis = y_axis / np.linalg.norm(y_axis)
            
            # 重新计算Z轴确保正交
            z_axis = np.cross(x_axis, y_axis)
            
            # 构建旋转矩阵
            rotation = np.column_stack([x_axis, y_axis, z_axis])
            
            # 原点设为两肩中点
            origin = (left_shoulder + right_shoulder) / 2
            
            # 构建T-pose变换矩阵
            self.t_pose_transform = np.eye(4)
            self.t_pose_transform[:3, :3] = rotation
            self.t_pose_transform[:3, 3] = origin
            
            print("T-pose标定成功!")
            print(f"T-pose变换矩阵:\n{self.t_pose_transform}")
            
            return True
        else:
            print("T-pose标定失败: 缺少肩膀关节位置")
            return False
    
    def save_calibration_result(self, filename: str = "coordinate_calibration.json"):
        """
        保存标定结果
        
        Args:
            filename: 保存文件名
        """
        calibration_data = {
            'robot_base_transform': self.robot_base_transform.tolist() if self.robot_base_transform is not None else None,
            't_pose_transform': self.t_pose_transform.tolist() if self.t_pose_transform is not None else None,
            't_pose_positions': {k: v.tolist() for k, v in self.t_pose_positions.items()},
            'calibration_points': [
                {
                    'camera_point': point['camera_point'].tolist(),
                    'robot_point': point['robot_point'].tolist(),
                    'name': point['name'],
                    'timestamp': point['timestamp']
                }
                for point in self.calibration_points
            ],
            'num_calibration_points': len(self.calibration_points)
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(calibration_data, f, indent=4)
            print(f"坐标系标定结果已保存到 {filename}")
        except Exception as e:
            print(f"保存标定结果时出错: {e}")
    
    def load_calibration_result(self, filename: str = "coordinate_calibration.json") -> bool:
        """
        加载标定结果
        
        Args:
            filename: 文件名
            
        Returns:
            bool: 是否成功加载
        """
        try:
            with open(filename, 'r') as f:
                calibration_data = json.load(f)
            
            if calibration_data['robot_base_transform']:
                self.robot_base_transform = np.array(calibration_data['robot_base_transform'])
            
            if calibration_data['t_pose_transform']:
                self.t_pose_transform = np.array(calibration_data['t_pose_transform'])
            
            self.t_pose_positions = {
                k: np.array(v) for k, v in calibration_data['t_pose_positions'].items()
            }
            
            print(f"坐标系标定结果已从 {filename} 加载")
            print(f"标定点数量: {calibration_data['num_calibration_points']}")
            
            return True
            
        except Exception as e:
            print(f"加载标定结果时出错: {e}")
            return False

def test_coordinate_system():
    """测试坐标系标定功能"""
    print("=== 坐标系标定测试 ===")
    
    # 创建相机对象
    camera = RealSenseCamera()
    
    # 启动相机
    if not camera.start_streaming():
        print("相机启动失败")
        return
    
    # 创建位姿检测器
    pose_detector = PoseDetector(camera)
    
    # 创建坐标系标定器
    coord_calibrator = CoordinateSystemCalibrator(camera, pose_detector)
    
    try:
        print("1. 坐标系标定")
        print("请准备ArUco标记或彩色目标物")
        
        # 捕获标定点
        points_to_capture = [
            "原点 (0,0,0)",
            "X轴正方向",
            "Y轴正方向",
            "Z轴正方向"
        ]
        
        for point_name in points_to_capture:
            input(f"按回车键开始捕获: {point_name}")
            if not coord_calibrator.capture_calibration_point_interactive(point_name):
                print(f"捕获{point_name}失败")
                break
        
        # 执行标定
        if len(coord_calibrator.calibration_points) >= 3:
            if coord_calibrator.calibrate_coordinate_system():
                # 保存结果
                coord_calibrator.save_calibration_result()
            else:
                print("坐标系标定失败")
        else:
            print("标定点数量不足")
        
        print("\n2. T-pose标定示例")
        # 这里可以添加T-pose标定的示例代码
        
    finally:
        # 清理
        camera.stop_streaming()
        cv2.destroyAllWindows()
    
    print("=== 测试完成 ===")

if __name__ == "__main__":
    test_coordinate_system()