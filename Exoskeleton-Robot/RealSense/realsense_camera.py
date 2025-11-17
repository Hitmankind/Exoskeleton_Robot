#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSense相机初始化和配置模块
用于机械臂视觉系统的相机控制和数据获取
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import json
import time
from typing import Tuple, Optional, Dict, Any

class RealSenseCamera:
    """RealSense相机控制类"""
    
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        """
        初始化RealSense相机
        
        Args:
            width: 图像宽度
            height: 图像高度  
            fps: 帧率
        """
        self.width = width
        self.height = height
        self.fps = fps
        
        # 创建pipeline和config
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # 配置流
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        
        # 对齐器：将深度图对齐到彩色图
        self.align = rs.align(rs.stream.color)
        
        # 相机内参
        self.intrinsics = None
        self.depth_scale = None
        
        # 标定参数
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rotation_vector = None
        self.translation_vector = None
        
        print("RealSense相机初始化完成")
    
    def start_streaming(self) -> bool:
        """
        开始数据流
        
        Returns:
            bool: 是否成功启动
        """
        try:
            # 启动pipeline
            profile = self.pipeline.start(self.config)
            
            # 获取深度传感器的深度比例
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            
            # 获取相机内参
            color_stream = profile.get_stream(rs.stream.color)
            self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics() # 出厂相机内参
            
            print(f"相机启动成功")
            print(f"深度比例: {self.depth_scale}")
            print(f"相机内参: {self.intrinsics}")
            
            return True
            
        except Exception as e:
            print(f"相机启动失败: {e}")
            return False
    
    def stop_streaming(self):
        """停止数据流"""
        try:
            self.pipeline.stop()
            print("相机数据流已停止")
        except Exception as e:
            print(f"停止相机时出错: {e}")
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        获取一帧彩色图和深度图
        
        Returns:
            Tuple[color_image, depth_image]: 彩色图和深度图
        """
        try:
            # 等待帧
            frames = self.pipeline.wait_for_frames()
            
            # 对齐帧
            aligned_frames = self.align.process(frames)
            
            # 获取对齐后的帧
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return None, None
            
            # 转换为numpy数组
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            return color_image, depth_image
            
        except Exception as e:
            print(f"获取帧时出错: {e}")
            return None, None
    
    def get_3d_point(self, pixel_x: int, pixel_y: int, depth_image: np.ndarray) -> Optional[Tuple[float, float, float]]:
        """
        根据像素坐标和深度值计算3D坐标
        
        Args:
            pixel_x: 像素x坐标
            pixel_y: 像素y坐标
            depth_image: 深度图
            
        Returns:
            Tuple[x, y, z]: 3D坐标（米）
        """
        if self.intrinsics is None:
            print("相机内参未初始化")
            return None
        
        try:
            # 获取深度值
            depth_value = depth_image[pixel_y, pixel_x]
            
            if depth_value == 0:
                return None
            
            # 转换为米
            depth_in_meters = depth_value * self.depth_scale
            
            # 使用相机内参计算3D坐标
            point_3d = rs.rs2_deproject_pixel_to_point(
                self.intrinsics, [pixel_x, pixel_y], depth_in_meters
            )
            
            return tuple(point_3d)
            
        except Exception as e:
            print(f"计算3D坐标时出错: {e}")
            return None
    
    def save_camera_info(self, filename: str = "camera_info.json"):
        """
        保存相机信息到文件
        
        Args:
            filename: 保存文件名
        """
        if self.intrinsics is None:
            print("相机内参未初始化")
            return
        
        camera_info = {
            "width": self.intrinsics.width,
            "height": self.intrinsics.height,
            "fx": self.intrinsics.fx,
            "fy": self.intrinsics.fy,
            "ppx": self.intrinsics.ppx,
            "ppy": self.intrinsics.ppy,
            "model": str(self.intrinsics.model),
            "coeffs": self.intrinsics.coeffs,
            "depth_scale": self.depth_scale
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(camera_info, f, indent=4)
            print(f"相机信息已保存到 {filename}")
        except Exception as e:
            print(f"保存相机信息时出错: {e}")
    
    def load_camera_info(self, filename: str = "camera_info.json") -> bool:
        """
        从文件加载相机信息
        
        Args:
            filename: 文件名
            
        Returns:
            bool: 是否成功加载
        """
        try:
            with open(filename, 'r') as f:
                camera_info = json.load(f)
            
            # 重建内参对象
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = camera_info["width"]
            self.intrinsics.height = camera_info["height"]
            self.intrinsics.fx = camera_info["fx"]
            self.intrinsics.fy = camera_info["fy"]
            self.intrinsics.ppx = camera_info["ppx"]
            self.intrinsics.ppy = camera_info["ppy"]
            self.intrinsics.coeffs = camera_info["coeffs"]
            self.depth_scale = camera_info["depth_scale"]
            
            print(f"相机信息已从 {filename} 加载")
            return True
            
        except Exception as e:
            print(f"加载相机信息时出错: {e}")
            return False
    
    def capture_image(self, save_path: str = None) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        捕获一张图像
        
        Args:
            save_path: 保存路径（可选）
            
        Returns:
            Tuple[color_image, depth_image]: 彩色图和深度图
        """
        color_image, depth_image = self.get_frames()
        
        if color_image is not None and depth_image is not None:
            if save_path:
                # 保存彩色图
                cv2.imwrite(f"{save_path}_color.jpg", color_image)
                
                # 保存深度图（转换为可视化格式）
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), 
                    cv2.COLORMAP_JET
                )
                cv2.imwrite(f"{save_path}_depth.jpg", depth_colormap)
                
                # 保存原始深度数据
                np.save(f"{save_path}_depth.npy", depth_image)
                
                print(f"图像已保存到 {save_path}")
        
        return color_image, depth_image
    
    def __del__(self):
        """析构函数"""
        try:
            self.stop_streaming()
        except:
            pass

def test_camera():
    """测试相机功能"""
    print("=== RealSense相机测试 ===")
    
    # 创建相机对象
    camera = RealSenseCamera()
    
    # 启动相机
    if not camera.start_streaming():
        print("相机启动失败")
        return
    
    try:
        # 等待相机稳定
        print("等待相机稳定...")
        time.sleep(2)
        
        # 捕获几帧图像
        for i in range(5):
            color_image, depth_image = camera.get_frames()
            
            if color_image is not None and depth_image is not None:
                print(f"第{i+1}帧: 彩色图 {color_image.shape}, 深度图 {depth_image.shape}")
                
                # 显示图像（可选）
                #cv2.imshow('Color Image', color_image)
                
                # 深度图可视化
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), 
                    cv2.COLORMAP_JET
                )
                #cv2.imshow('Depth Image', depth_colormap)
                
                # 按ESC退出
                #if cv2.waitKey(1) & 0xFF == 27:
                #    break
            else:
                print(f"第{i+1}帧获取失败")
        
        # 保存相机信息
        camera.save_camera_info("camera_info.json")
        
        # 捕获测试图像
        camera.capture_image("test_capture")
        
    finally:
        # 清理
        camera.stop_streaming()
        #cv2.destroyAllWindows()
    
    print("=== 测试完成 ===")

if __name__ == "__main__":
    test_camera()
