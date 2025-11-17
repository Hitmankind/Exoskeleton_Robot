#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
目标位姿检测模块
用于通过RealSense相机检测目标物体并计算其位姿
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List, Dict, Any
from realsense_camera import RealSenseCamera
import json
import time

class PoseDetector:
    """目标位姿检测器"""
    
    def __init__(self, camera: RealSenseCamera):
        """
        初始化位姿检测器
        
        Args:
            camera: RealSense相机对象
        """
        self.camera = camera
        
        # ArUco标记检测器
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # 颜色检测参数
        self.color_ranges = {
            'red': {
                'lower': np.array([0, 50, 50]),
                'upper': np.array([10, 255, 255])
            },
            'green': {
                'lower': np.array([40, 50, 50]),
                'upper': np.array([80, 255, 255])
            },
            'blue': {
                'lower': np.array([100, 50, 50]),
                'upper': np.array([130, 255, 255])
            }
        }
        
        # 目标检测结果
        self.last_detection = None
        
        print("位姿检测器初始化完成")
    
    def detect_aruco_markers(self, color_image: np.ndarray, depth_image: np.ndarray, 
                           marker_size: float = 0.05) -> List[Dict[str, Any]]:
        """
        检测ArUco标记并计算位姿
        
        Args:
            color_image: 彩色图像
            depth_image: 深度图像
            marker_size: 标记实际尺寸（米）
            
        Returns:
            List[Dict]: 检测到的标记信息列表
        """
        if self.camera.intrinsics is None:
            print("相机内参未初始化")
            return []
        
        # 转换为灰度图
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
        # 检测ArUco标记
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        markers = []
        
        if ids is not None:
            # 构建相机矩阵
            camera_matrix = np.array([
                [self.camera.intrinsics.fx, 0, self.camera.intrinsics.ppx],
                [0, self.camera.intrinsics.fy, self.camera.intrinsics.ppy],
                [0, 0, 1]
            ])
            
            # 畸变系数
            dist_coeffs = np.array(self.camera.intrinsics.coeffs)
            
            # 估计每个标记的位姿
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_size, camera_matrix, dist_coeffs
            )
            
            for i, marker_id in enumerate(ids.flatten()):
                # 获取标记中心点的3D坐标
                center_2d = np.mean(corners[i][0], axis=0).astype(int)
                center_3d = self.camera.get_3d_point(center_2d[0], center_2d[1], depth_image)
                
                if center_3d is not None:
                    marker_info = {
                        'id': int(marker_id),
                        'corners_2d': corners[i][0].tolist(),
                        'center_2d': center_2d.tolist(),
                        'center_3d': center_3d,
                        'rvec': rvecs[i][0].tolist(),
                        'tvec': tvecs[i][0].tolist(),
                        'rotation_matrix': cv2.Rodrigues(rvecs[i][0])[0].tolist(),
                        'pose_matrix': self._create_pose_matrix(rvecs[i][0], tvecs[i][0])
                    }
                    markers.append(marker_info)
        
        return markers
    
    def detect_color_objects(self, color_image: np.ndarray, depth_image: np.ndarray, 
                           target_color: str = 'red') -> List[Dict[str, Any]]:
        """
        基于颜色检测目标物体
        
        Args:
            color_image: 彩色图像
            depth_image: 深度图像
            target_color: 目标颜色 ('red', 'green', 'blue')
            
        Returns:
            List[Dict]: 检测到的物体信息列表
        """
        if target_color not in self.color_ranges:
            print(f"不支持的颜色: {target_color}")
            return []
        
        # 转换到HSV色彩空间
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # 颜色阈值分割
        color_range = self.color_ranges[target_color]
        mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
        
        # 形态学操作去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        objects = []
        
        for contour in contours:
            # 过滤小轮廓
            area = cv2.contourArea(contour)
            if area < 500:  # 最小面积阈值
                continue
            
            # 计算轮廓的边界框和中心
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w // 2
            center_y = y + h // 2
            
            # 获取3D坐标
            center_3d = self.camera.get_3d_point(center_x, center_y, depth_image)
            
            if center_3d is not None:
                # 计算轮廓的最小外接矩形（用于估计方向）
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                
                object_info = {
                    'color': target_color,
                    'area': area,
                    'bbox': [x, y, w, h],
                    'center_2d': [center_x, center_y],
                    'center_3d': center_3d,
                    'contour': contour.tolist(),
                    'min_area_rect': {
                        'center': rect[0],
                        'size': rect[1],
                        'angle': rect[2]
                    },
                    'oriented_box': box.tolist()
                }
                objects.append(object_info)
        
        return objects
    
    def detect_target_pose(self, detection_method: str = 'aruco', **kwargs) -> Optional[np.ndarray]:
        """
        检测目标位姿
        
        Args:
            detection_method: 检测方法 ('aruco', 'color')
            **kwargs: 其他参数
            
        Returns:
            np.ndarray: 4x4位姿矩阵，如果检测失败返回None
        """
        # 获取图像
        color_image, depth_image = self.camera.get_frames()
        
        if color_image is None or depth_image is None:
            print("无法获取图像")
            return None
        
        target_pose = None
        
        if detection_method == 'aruco':
            # ArUco标记检测
            marker_size = kwargs.get('marker_size', 0.05)
            markers = self.detect_aruco_markers(color_image, depth_image, marker_size)
            
            if markers:
                # 使用第一个检测到的标记
                target_pose = np.array(markers[0]['pose_matrix'])
                self.last_detection = {
                    'method': 'aruco',
                    'data': markers[0],
                    'timestamp': time.time()
                }
                print(f"检测到ArUco标记 ID: {markers[0]['id']}")
        
        elif detection_method == 'color':
            # 颜色检测
            target_color = kwargs.get('color', 'red')
            objects = self.detect_color_objects(color_image, depth_image, target_color)
            
            if objects:
                # 使用面积最大的物体
                largest_object = max(objects, key=lambda x: x['area'])
                
                # 为颜色检测创建简单的位姿矩阵（只有位置，方向为单位矩阵）
                x, y, z = largest_object['center_3d']
                target_pose = np.array([
                    [1, 0, 0, x],
                    [0, 1, 0, y],
                    [0, 0, 1, z],
                    [0, 0, 0, 1]
                ])
                
                self.last_detection = {
                    'method': 'color',
                    'data': largest_object,
                    'timestamp': time.time()
                }
                print(f"检测到{target_color}色物体，面积: {largest_object['area']}")
        
        return target_pose
    
    def get_target_pose_for_kinematics(self, detection_method: str = 'aruco', **kwargs) -> Optional[np.ndarray]:
        """
        获取用于运动学计算的目标位姿
        
        Args:
            detection_method: 检测方法
            **kwargs: 其他参数
            
        Returns:
            np.ndarray: 4x4位姿矩阵，适用于运动学计算
        """
        pose = self.detect_target_pose(detection_method, **kwargs)
        
        if pose is not None:
            # 这里可以添加坐标系转换逻辑
            # 将相机坐标系转换为机械臂基座标系
            # 目前返回原始位姿，后续可以根据标定结果进行转换
            return pose
        
        return None
    
    def visualize_detection(self, color_image: np.ndarray, depth_image: np.ndarray, 
                          detection_method: str = 'aruco', **kwargs) -> np.ndarray:
        """
        可视化检测结果
        
        Args:
            color_image: 彩色图像
            depth_image: 深度图像
            detection_method: 检测方法
            **kwargs: 其他参数
            
        Returns:
            np.ndarray: 带有检测结果的图像
        """
        result_image = color_image.copy()
        
        if detection_method == 'aruco':
            marker_size = kwargs.get('marker_size', 0.05)
            markers = self.detect_aruco_markers(color_image, depth_image, marker_size)
            
            for marker in markers:
                # 绘制标记边界
                corners = np.array(marker['corners_2d'], dtype=np.int32)
                cv2.polylines(result_image, [corners], True, (0, 255, 0), 2)
                
                # 绘制标记ID
                center = tuple(marker['center_2d'])
                cv2.putText(result_image, f"ID: {marker['id']}", 
                          (center[0]-20, center[1]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # 绘制3D坐标
                coord_text = f"({marker['center_3d'][0]:.3f}, {marker['center_3d'][1]:.3f}, {marker['center_3d'][2]:.3f})"
                cv2.putText(result_image, coord_text, 
                          (center[0]-50, center[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        
        elif detection_method == 'color':
            target_color = kwargs.get('color', 'red')
            objects = self.detect_color_objects(color_image, depth_image, target_color)
            
            for obj in objects:
                # 绘制边界框
                x, y, w, h = obj['bbox']
                cv2.rectangle(result_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                # 绘制中心点
                center = tuple(obj['center_2d'])
                cv2.circle(result_image, center, 5, (0, 0, 255), -1)
                
                # 绘制方向框
                box = np.array(obj['oriented_box'], dtype=np.int32)
                cv2.drawContours(result_image, [box], 0, (255, 0, 0), 2)
                
                # 绘制信息
                info_text = f"{obj['color']} Area: {obj['area']}"
                cv2.putText(result_image, info_text, 
                          (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                coord_text = f"({obj['center_3d'][0]:.3f}, {obj['center_3d'][1]:.3f}, {obj['center_3d'][2]:.3f})"
                cv2.putText(result_image, coord_text, 
                          (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        
        return result_image
    
    def _create_pose_matrix(self, rvec: np.ndarray, tvec: np.ndarray) -> List[List[float]]:
        """
        从旋转向量和平移向量创建4x4位姿矩阵
        
        Args:
            rvec: 旋转向量
            tvec: 平移向量
            
        Returns:
            List[List[float]]: 4x4位姿矩阵
        """
        # 转换旋转向量为旋转矩阵
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # 创建4x4位姿矩阵
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = rotation_matrix
        pose_matrix[:3, 3] = tvec.flatten()
        
        return pose_matrix.tolist()
    
    def save_detection_result(self, filename: str = "detection_result.json"):
        """
        保存最后一次检测结果
        
        Args:
            filename: 保存文件名
        """
        if self.last_detection is None:
            print("没有检测结果可保存")
            return
        
        try:
            with open(filename, 'w') as f:
                json.dump(self.last_detection, f, indent=4)
            print(f"检测结果已保存到 {filename}")
        except Exception as e:
            print(f"保存检测结果时出错: {e}")

def test_pose_detection():
    """测试位姿检测功能"""
    print("=== 位姿检测测试 ===")
    
    # 创建相机对象
    camera = RealSenseCamera()
    
    # 启动相机
    if not camera.start_streaming():
        print("相机启动失败")
        return
    
    # 创建位姿检测器
    detector = PoseDetector(camera)
    
    try:
        print("开始检测，按ESC退出...")
        
        while True:
            # 获取图像
            color_image, depth_image = camera.get_frames()
            
            if color_image is not None and depth_image is not None:
                # ArUco检测
                aruco_result = detector.visualize_detection(color_image, depth_image, 'aruco')
                cv2.imshow('ArUco Detection', aruco_result)
                
                # 颜色检测
                color_result = detector.visualize_detection(color_image, depth_image, 'color', color='red')
                cv2.imshow('Color Detection', color_result)
                
                # 检测目标位姿
                pose = detector.get_target_pose_for_kinematics('aruco')
                if pose is not None:
                    print("检测到目标位姿:")
                    print(pose)
            
            # 按ESC退出
            if cv2.waitKey(1) & 0xFF == 27:
                break
    
    finally:
        # 清理
        camera.stop_streaming()
        cv2.destroyAllWindows()
        
        # 保存最后的检测结果
        #detector.save_detection_result("last_detection.json")
    
    print("=== 测试完成 ===")

if __name__ == "__main__":
    test_pose_detection()