#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机标定模块
包括相机内参标定和手眼标定功能
"""

import cv2
import numpy as np
import json
import os
from typing import List, Tuple, Optional, Dict, Any
from realsense_camera import RealSenseCamera
import time

class CameraCalibrator:
    """相机标定器"""
    
    def __init__(self, camera: RealSenseCamera):
        """
        初始化标定器
        
        Args:
            camera: RealSense相机对象
        """
        self.camera = camera
        
        # 棋盘格参数
        self.chessboard_size = (6, 6)  # 内角点数量 (宽, 高)
        self.square_size = 0.0144  # 棋盘格方格大小（米）
        
        # 标定数据
        self.calibration_images = []
        self.object_points = []  # 3D点
        self.image_points = []   # 2D点
        
        # 标定结果
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_error = None
        
        # 手眼标定数据
        self.robot_poses = []  # 机器人位姿
        self.camera_poses = []  # 相机位姿
        self.hand_eye_transform = None  # 手眼变换矩阵
        
        print("相机标定器初始化完成")
    
    def prepare_object_points(self) -> np.ndarray:
        """
        准备棋盘格的3D坐标点
        
        Returns:
            np.ndarray: 3D坐标点
        """
        # 创建棋盘格角点的3D坐标
        objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        
        return objp
    
    def capture_calibration_images(self, num_images: int = 20, save_dir: str = "calibration_images") -> bool:
        """
        捕获标定图像
        
        Args:
            num_images: 需要捕获的图像数量
            save_dir: 保存目录
            
        Returns:
            bool: 是否成功捕获足够的图像
        """
        # 创建保存目录
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        
        print(f"开始捕获{num_images}张标定图像...")
        print("将棋盘格放在相机前，按空格键捕获图像，按ESC退出")
        
        captured_count = 0
        objp = self.prepare_object_points()
        
        while captured_count < num_images:
            # 获取图像
            color_image, _ = self.camera.get_frames()
            
            if color_image is None:
                continue
            
            # 转换为灰度图
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            
            # 查找棋盘格角点
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
            
            # 显示图像
            display_image = color_image.copy()
            
            if ret:
                # 精确化角点位置
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                # 绘制角点
                cv2.drawChessboardCorners(display_image, self.chessboard_size, corners_refined, ret)
                
                # 显示状态
                status_text = f"Success to Detect CheckerBoard! press SPACE capture ({captured_count}/{num_images})"
                cv2.putText(display_image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                status_text = f"Fail to Detect CheckerBoard ({captured_count}/{num_images})"
                cv2.putText(display_image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow('Camera Calibration', display_image)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' ') and ret:  # 空格键捕获
                # 保存图像和角点
                image_filename = os.path.join(save_dir, f"calibration_{captured_count:02d}.jpg")
                cv2.imwrite(image_filename, color_image)
                
                self.calibration_images.append(color_image)
                self.object_points.append(objp)
                self.image_points.append(corners_refined)
                
                captured_count += 1
                print(f"捕获第{captured_count}张图像")
                
                # 短暂延时避免重复捕获
                time.sleep(0.5)
            
            elif key == 27:  # ESC退出
                break
        
        cv2.destroyAllWindows()
        
        success = captured_count >= 10  # 至少需要10张图像
        if success:
            print(f"成功捕获{captured_count}张标定图像")
        else:
            print(f"捕获的图像数量不足: {captured_count}/10")
        
        return success
    
    def calibrate_camera(self) -> bool:
        """
        执行相机内参标定
        
        Returns:
            bool: 标定是否成功
        """
        if len(self.object_points) < 10:
            print("标定图像数量不足，请先捕获足够的图像")
            return False
        
        print("开始相机内参标定...")
        
        # 获取图像尺寸
        if self.calibration_images:
            h, w = self.calibration_images[0].shape[:2]
        else:
            h, w = self.camera.height, self.camera.width
        
        # 执行标定
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.object_points, self.image_points, (w, h), None, None
        )
        
        if ret:
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist_coeffs
            
            # 计算重投影误差
            total_error = 0
            for i in range(len(self.object_points)):
                imgpoints2, _ = cv2.projectPoints(
                    self.object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
                )
                error = cv2.norm(self.image_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                total_error += error
            
            self.calibration_error = total_error / len(self.object_points)
            
            print(f"相机标定成功!")
            print(f"重投影误差: {self.calibration_error:.4f} 像素")
            print(f"相机矩阵:\n{camera_matrix}")
            print(f"畸变系数: {dist_coeffs.flatten()}")
            
            return True
        else:
            print("相机标定失败")
            return False
    
    def save_calibration_result(self, filename: str = "camera_calibration.json"):
        """
        保存标定结果
        
        Args:
            filename: 保存文件名
        """
        if self.camera_matrix is None:
            print("没有标定结果可保存")
            return
        
        calibration_data = {
            'camera_matrix': self.camera_matrix.tolist(),
            'dist_coeffs': self.dist_coeffs.tolist(),
            'calibration_error': float(self.calibration_error),
            'image_size': [self.camera.width, self.camera.height],
            'chessboard_size': self.chessboard_size,
            'square_size': self.square_size,
            'num_images': len(self.calibration_images)
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(calibration_data, f, indent=4)
            print(f"标定结果已保存到 {filename}")
        except Exception as e:
            print(f"保存标定结果时出错: {e}")
    
    def load_calibration_result(self, filename: str = "camera_calibration.json") -> bool:
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
            
            self.camera_matrix = np.array(calibration_data['camera_matrix'])
            self.dist_coeffs = np.array(calibration_data['dist_coeffs'])
            self.calibration_error = calibration_data['calibration_error']
            
            print(f"标定结果已从 {filename} 加载")
            print(f"重投影误差: {self.calibration_error:.4f} 像素")
            
            return True
            
        except Exception as e:
            print(f"加载标定结果时出错: {e}")
            return False
    
    def add_hand_eye_sample(self, robot_pose: np.ndarray, target_pose: np.ndarray):
        """
        添加手眼标定样本
        
        Args:
            robot_pose: 机器人末端位姿 (4x4矩阵)
            target_pose: 相机观测到的目标位姿 (4x4矩阵)
        """
        self.robot_poses.append(robot_pose.copy())
        self.camera_poses.append(target_pose.copy())
        
        print(f"添加手眼标定样本，当前样本数: {len(self.robot_poses)}")
    
    def calibrate_hand_eye(self, method: str = 'tsai') -> bool:
        """
        执行手眼标定
        
        Args:
            method: 标定方法 ('tsai', 'park', 'horaud', 'andreff', 'daniilidis')
            
        Returns:
            bool: 标定是否成功
        """
        if len(self.robot_poses) < 3:
            print("手眼标定样本数量不足，至少需要3个样本")
            return False
        
        print(f"开始手眼标定，使用{method}方法...")
        
        # 转换为OpenCV格式
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []
        
        for robot_pose, camera_pose in zip(self.robot_poses, self.camera_poses):
            # 机器人位姿
            R_gripper2base.append(robot_pose[:3, :3])
            t_gripper2base.append(robot_pose[:3, 3])
            
            # 相机位姿（需要求逆，因为我们要的是相机到目标的变换）
            camera_pose_inv = np.linalg.inv(camera_pose)
            R_target2cam.append(camera_pose_inv[:3, :3])
            t_target2cam.append(camera_pose_inv[:3, 3])
        
        # 选择标定方法
        method_map = {
            'tsai': cv2.CALIB_HAND_EYE_TSAI,
            'park': cv2.CALIB_HAND_EYE_PARK,
            'horaud': cv2.CALIB_HAND_EYE_HORAUD,
            'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
            'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS
        }
        
        if method not in method_map:
            print(f"不支持的标定方法: {method}")
            return False
        
        try:
            # 执行手眼标定
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base,
                R_target2cam, t_target2cam,
                method=method_map[method]
            )
            
            # 构建手眼变换矩阵
            self.hand_eye_transform = np.eye(4)
            self.hand_eye_transform[:3, :3] = R_cam2gripper
            self.hand_eye_transform[:3, 3] = t_cam2gripper.flatten()
            
            print("手眼标定成功!")
            print(f"手眼变换矩阵:\n{self.hand_eye_transform}")
            
            return True
            
        except Exception as e:
            print(f"手眼标定失败: {e}")
            return False
    
    def save_hand_eye_result(self, filename: str = "hand_eye_calibration.json"):
        """
        保存手眼标定结果
        
        Args:
            filename: 保存文件名
        """
        if self.hand_eye_transform is None:
            print("没有手眼标定结果可保存")
            return
        
        hand_eye_data = {
            'hand_eye_transform': self.hand_eye_transform.tolist(),
            'num_samples': len(self.robot_poses),
            'robot_poses': [pose.tolist() for pose in self.robot_poses],
            'camera_poses': [pose.tolist() for pose in self.camera_poses]
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(hand_eye_data, f, indent=4)
            print(f"手眼标定结果已保存到 {filename}")
        except Exception as e:
            print(f"保存手眼标定结果时出错: {e}")
    
    def load_hand_eye_result(self, filename: str = "hand_eye_calibration.json") -> bool:
        """
        加载手眼标定结果
        
        Args:
            filename: 文件名
            
        Returns:
            bool: 是否成功加载
        """
        try:
            with open(filename, 'r') as f:
                hand_eye_data = json.load(f)
            
            self.hand_eye_transform = np.array(hand_eye_data['hand_eye_transform'])
            
            print(f"手眼标定结果已从 {filename} 加载")
            print(f"样本数量: {hand_eye_data['num_samples']}")
            
            return True
            
        except Exception as e:
            print(f"加载手眼标定结果时出错: {e}")
            return False
    
    def transform_pose_to_robot_frame(self, camera_pose: np.ndarray) -> Optional[np.ndarray]:
        """
        将相机坐标系中的位姿转换到机器人基座标系
        
        Args:
            camera_pose: 相机坐标系中的位姿
            
        Returns:
            np.ndarray: 机器人基座标系中的位姿
        """
        if self.hand_eye_transform is None:
            print("手眼标定结果未加载")
            return None
        
        # 这里需要根据具体的手眼标定配置进行转换
        # 简化版本，实际使用时需要根据标定结果调整
        robot_pose = self.hand_eye_transform @ camera_pose
        
        return robot_pose

def test_camera_calibration():
    """测试相机标定功能"""
    print("=== 相机标定测试 ===")
    
    # 创建相机对象
    camera = RealSenseCamera()
    
    # 启动相机
    if not camera.start_streaming():
        print("相机启动失败")
        return
    
    # 创建标定器
    calibrator = CameraCalibrator(camera)
    
    try:
        print("1. 相机内参标定")
        print("请准备9x6的棋盘格标定板")
        input("按回车键开始捕获标定图像...")
        
        # 捕获标定图像
        if calibrator.capture_calibration_images(num_images=15):
            # 执行标定
            if calibrator.calibrate_camera():
                # 保存结果
                calibrator.save_calibration_result()
            else:
                print("相机标定失败")
        else:
            print("标定图像捕获失败")
    
    finally:
        # 清理
        camera.stop_streaming()
        cv2.destroyAllWindows()
    
    print("=== 测试完成 ===")

if __name__ == "__main__":
    test_camera_calibration()