#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSense机械臂系统主程序
集成相机控制、位姿检测、标定和坐标系转换功能
"""

import cv2
import numpy as np
import json
import time
import os
from typing import Optional, Dict, List, Tuple

# 导入自定义模块
from realsense_camera import RealSenseCamera
from pose_detection import PoseDetector
from camera_calibration import CameraCalibrator
from coordinate_system import CoordinateSystemCalibrator

import hand_eye_calibration

class RealSenseRobotSystem:
    """RealSense机械臂系统主类"""
    
    def __init__(self):
        """初始化系统"""
        print("=== RealSense机械臂系统初始化 ===")
        
        # 初始化组件
        self.camera = None
        self.pose_detector = None
        self.camera_calibrator = None
        self.coord_calibrator = None
        
        # 系统状态
        self.is_initialized = False
        self.is_calibrated = False
        
        print("系统初始化完成")
    
    def initialize_system(self) -> bool:
        """
        初始化系统组件
        
        Returns:
            bool: 初始化是否成功
        """
        try:
            print("正在初始化RealSense相机...")
            
            # 初始化相机
            self.camera = RealSenseCamera()
            if not self.camera.start_streaming():
                print("相机启动失败")
                return False
            
            # 初始化位姿检测器
            print("正在初始化位姿检测器...")
            self.pose_detector = PoseDetector(self.camera)
            
            # 初始化标定器
            print("正在初始化标定器...")
            self.camera_calibrator = CameraCalibrator(self.camera)
            self.coord_calibrator = CoordinateSystemCalibrator(self.camera, self.pose_detector)
            
            # 尝试加载已有的标定结果
            self.load_calibration_data()
            
            self.is_initialized = True
            print("系统初始化成功!")
            
            return True
            
        except Exception as e:
            print(f"系统初始化失败: {e}")
            return False
    
    def load_calibration_data(self):
        """加载标定数据"""
        print("正在加载标定数据...")
        
        # 加载相机标定结果
        if os.path.exists("camera_calibration.json"):
            if self.camera_calibrator.load_calibration_result():
                print("相机标定数据加载成功")
        
        # 加载坐标系标定结果
        if os.path.exists("coordinate_calibration.json"):
            if self.coord_calibrator.load_calibration_result():
                print("坐标系标定数据加载成功")
                self.is_calibrated = True
    
    def camera_calibration_workflow(self) -> bool:
        """
        相机标定工作流程
        
        Returns:
            bool: 标定是否成功
        """
        print("\n=== 相机标定工作流程 ===")
        
        if not self.is_initialized:
            print("系统未初始化")
            return False
        
        print("请准备9x6的棋盘格标定板")
        input("按回车键开始标定...")
        
        # 捕获标定图像
        if self.camera_calibrator.capture_calibration_images(num_images=15):
            # 执行标定
            if self.camera_calibrator.calibrate_camera():
                # 保存结果
                self.camera_calibrator.save_calibration_result()
                print("相机标定完成!")
                return True
            else:
                print("相机标定失败")
                return False
        else:
            print("标定图像捕获失败")
            return False
    
    def coordinate_calibration_workflow(self) -> bool:
        """
        坐标系标定工作流程
        
        Returns:
            bool: 标定是否成功
        """
        print("\n=== 坐标系标定工作流程 ===")
        
        if not self.is_initialized:
            print("系统未初始化")
            return False
        
        print("请准备ArUco标记或彩色目标物")
        
        # 定义标定点
        calibration_points = [
            "机器人基座原点 (0,0,0)",
            "X轴正方向 (例如: 0.1,0,0)",
            "Y轴正方向 (例如: 0,0.1,0)",
            "Z轴正方向 (例如: 0,0,0.1)",
            "验证点1 (例如: 0.05,0.05,0.05)"
        ]
        
        # 捕获标定点
        for point_name in calibration_points:
            print(f"\n准备捕获: {point_name}")
            input("按回车键开始...")
            
            if not self.coord_calibrator.capture_calibration_point_interactive(point_name):
                print(f"捕获{point_name}失败")
                continue
        
        # 执行标定
        if len(self.coord_calibrator.calibration_points) >= 3:
            if self.coord_calibrator.calibrate_coordinate_system():
                # 保存结果
                self.coord_calibrator.save_calibration_result()
                self.is_calibrated = True
                print("坐标系标定完成!")
                return True
            else:
                print("坐标系标定失败")
                return False
        else:
            print("标定点数量不足")
            return False
    
    def real_time_pose_detection(self):
        """实时位姿检测"""
        print("\n=== 实时位姿检测 ===")
        print("按ESC退出，按空格键保存当前检测结果")
        
        if not self.is_initialized:
            print("系统未初始化")
            return
        
        detection_count = 0
        
        while True:
            # 获取图像
            color_image, depth_image = self.camera.get_frames()
            
            if color_image is None:
                continue
            
            # 检测目标
            detections = self.pose_detector.detect_targets(color_image, depth_image)
            
            # 显示图像
            display_image = color_image.copy()
            
            # 绘制检测结果
            for i, detection in enumerate(detections):
                # 绘制边界框
                bbox = detection.get('bbox')
                if bbox:
                    x, y, w, h = bbox
                    cv2.rectangle(display_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    
                    # 显示目标信息
                    target_type = detection.get('type', 'unknown')
                    cv2.putText(display_image, f"Target {i}: {target_type}", 
                              (x, y-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # 显示3D位置
                position = detection.get('position')
                if position is not None:
                    # 相机坐标系位置
                    pos_text = f"Cam: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
                    cv2.putText(display_image, pos_text, (x, y-10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    
                    # 如果已标定，显示机器人坐标系位置
                    if self.is_calibrated:
                        try:
                            robot_pos = self.coord_calibrator.transform_points(position.reshape(1, -1))[0]
                            robot_text = f"Robot: ({robot_pos[0]:.3f}, {robot_pos[1]:.3f}, {robot_pos[2]:.3f})"
                            cv2.putText(display_image, robot_text, (x, y+h+20), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                        except:
                            pass
            
            # 显示状态信息
            status_text = f"检测到 {len(detections)} 个目标"
            if self.is_calibrated:
                status_text += " | 坐标系已标定"
            else:
                status_text += " | 坐标系未标定"
            
            cv2.putText(display_image, status_text, (10, 30), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.putText(display_image, "ESC: 退出 | SPACE: 保存检测结果", (10, display_image.shape[0]-20), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            cv2.imshow('Real-time Pose Detection', display_image)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27:  # ESC退出
                break
            elif key == ord(' ') and detections:  # 空格键保存
                # 保存检测结果
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                result_file = f"detection_result_{timestamp}.json"
                
                result_data = {
                    'timestamp': timestamp,
                    'detections': []
                }
                
                for detection in detections:
                    detection_data = {
                        'type': detection.get('type', 'unknown'),
                        'camera_position': detection.get('position', []).tolist() if detection.get('position') is not None else None,
                        'bbox': detection.get('bbox'),
                        'confidence': detection.get('confidence', 0.0)
                    }
                    
                    # 如果已标定，添加机器人坐标
                    if self.is_calibrated and detection.get('position') is not None:
                        try:
                            robot_pos = self.coord_calibrator.transform_points(
                                detection['position'].reshape(1, -1)
                            )[0]
                            detection_data['robot_position'] = robot_pos.tolist()
                        except:
                            pass
                    
                    result_data['detections'].append(detection_data)
                
                # 保存到文件
                try:
                    with open(result_file, 'w') as f:
                        json.dump(result_data, f, indent=4)
                    
                    detection_count += 1
                    print(f"检测结果已保存到 {result_file}")
                    
                    # 在图像上显示保存提示
                    cv2.putText(display_image, f"Saved! ({detection_count})", (10, 60), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.imshow('Real-time Pose Detection', display_image)
                    cv2.waitKey(500)  # 显示0.5秒
                    
                except Exception as e:
                    print(f"保存检测结果失败: {e}")
        
        cv2.destroyAllWindows()
    
    def get_target_pose_for_robot(self, target_type: str = "aruco") -> Optional[Dict]:
        """
        获取目标位姿供机器人使用
        
        Args:
            target_type: 目标类型
            
        Returns:
            Dict: 目标位姿信息
        """
        if not self.is_initialized:
            print("系统未初始化")
            return None
        
        if not self.is_calibrated:
            print("坐标系未标定，无法提供机器人坐标")
            return None
        
        # 获取图像
        color_image, depth_image = self.camera.get_frames()
        
        if color_image is None:
            return None
        
        # 检测目标
        detections = self.pose_detector.detect_targets(color_image, depth_image)
        
        # 筛选指定类型的目标
        target_detections = [d for d in detections if d.get('type') == target_type]
        
        if not target_detections:
            return None
        
        # 选择置信度最高的目标
        best_detection = max(target_detections, key=lambda x: x.get('confidence', 0))
        
        # 转换到机器人坐标系
        camera_position = best_detection.get('position')
        if camera_position is None:
            return None
        
        try:
            robot_position = self.coord_calibrator.transform_points(
                camera_position.reshape(1, -1)
            )[0]
            
            # 构建返回结果
            result = {
                'type': best_detection.get('type'),
                'camera_position': camera_position.tolist(),
                'robot_position': robot_position.tolist(),
                'confidence': best_detection.get('confidence', 0.0),
                'bbox': best_detection.get('bbox'),
                'timestamp': time.time()
            }
            
            return result
            
        except Exception as e:
            print(f"坐标转换失败: {e}")
            return None
    
    def shutdown(self):
        """关闭系统"""
        print("正在关闭系统...")
        
        if self.camera:
            self.camera.stop_streaming()
        
        cv2.destroyAllWindows()
        print("系统已关闭")

def main():
    """主函数"""
    print("=== RealSense机械臂系统 ===")
    
    # 创建系统实例
    system = RealSenseRobotSystem()
    
    try:
        # 初始化系统
        if not system.initialize_system():
            print("系统初始化失败")
            return
        
        while True:
            print("\n=== 主菜单 ===")
            print("1. 相机内参标定")
            #print("2. 坐标系标定")
            print("2. 手眼标定")
            print("3. 实时位姿检测")
            print("4. 获取目标位姿（单次）")
            print("5. 系统状态")
            print("0. 退出")
            
            choice = input("请选择功能 (0-5): ").strip()
            
            if choice == '1':
                system.camera_calibration_workflow()
            
            elif choice == '2':
                #system.coordinate_calibration_workflow()
                # 停止 system 的相机流，否则手眼标定程序会启动自己的相机流并失败
                system.shutdown()
                # 执行手眼标定
                hand_eye_calibration.main()
                # 恢复初始化 system 
                system.initialize_system()
                
            elif choice == '3':
                system.real_time_pose_detection()
            
            elif choice == '4':
                target_type = input("请输入目标类型 (aruco/color): ").strip()
                if not target_type:
                    target_type = "aruco"
                
                result = system.get_target_pose_for_robot(target_type)
                if result:
                    print("检测到目标:")
                    print(f"  类型: {result['type']}")
                    print(f"  相机坐标: {result['camera_position']}")
                    print(f"  机器人坐标: {result['robot_position']}")
                    print(f"  置信度: {result['confidence']:.3f}")
                else:
                    print("未检测到目标")
            
            elif choice == '5':
                print(f"\n=== 系统状态 ===")
                print(f"系统初始化: {'是' if system.is_initialized else '否'}")
                print(f"坐标系标定: {'是' if system.is_calibrated else '否'}")
                
                if system.camera_calibrator and system.camera_calibrator.camera_matrix is not None:
                    print(f"相机标定: 是 (误差: {system.camera_calibrator.calibration_error:.4f})")
                else:
                    print("相机标定: 否")
                
                if system.coord_calibrator:
                    print(f"标定点数量: {len(system.coord_calibrator.calibration_points)}")
            
            elif choice == '0':
                break
            
            else:
                print("无效选择，请重新输入")
    
    except KeyboardInterrupt:
        print("\n用户中断")
    
    except Exception as e:
        print(f"系统错误: {e}")
    
    finally:
        # 清理资源
        system.shutdown()

if __name__ == "__main__":
    main()