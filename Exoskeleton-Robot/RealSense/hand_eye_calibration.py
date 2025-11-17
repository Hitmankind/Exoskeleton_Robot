from realsense_camera import RealSenseCamera
from camera_calibration import CameraCalibrator
from pose_detection import PoseDetector

import sys
sys.path.append("../IK&FK")
from kinematics_solver_modified import(
    initialize_servos, 
    forward_kinematics, 
    setPWMServoAngleWithTracking
)

import time
import cv2
import numpy as np

def main():
    print("=== RealSense手眼标定 ===")
    # 创建相机对象
    camera = RealSenseCamera()

    # 启动相机
    if not camera.start_streaming():
        print("[ERROR] 相机启动失败")
        return
    print("等待相机稳定...")
    time.sleep(2)

    # 加载相机内参
    # .........

    # 创建标定器（含手眼标定）
    calibrator = CameraCalibrator(camera)

    # 创建位姿检测器（计算标定物相对于相机的位姿）
    detector = PoseDetector(camera)

    # 初始化舵机
    time.sleep(1)
    initialize_servos()

    input("[SUCCESS] 准备阶段完成，请按 Enter 键继续...")
    try:
        print("开始进行手眼标定...")
        count = 0
        # 提供多组舵机角度用于标定
        joint_angles_list = [[90,90,90],[100,80,105],[105,100,105],[75,105,120],[77,91,94],[80,100,130],[100,88,140],[120,90,110],[90,120,0],[90,90,180]]
        for i in range(0,len(joint_angles_list)):

            robot_pose = [] # 机器人末端位姿（相对于机器人基座）
            target_pose = [] # 标定物目标位姿（相对于相机）

            # -------------------------------------------------------
            # 通过set舵机角度来移动机器人并计算robot_pose
            # -------------------------------------------------------
            time.sleep(2)
            # 设置舵机角度然后计算正运动学
            robot_pose = forward_kinematics(joint_angles_list[i])
            time.sleep(1)


            # ------------------------------------------
            # 获取图像并计算标定物相对于相机的位姿target_pose
            # ------------------------------------------
            # 获取彩色图和深度图
            color_image, depth_image = camera.get_frames()

            if color_image is not None and depth_image is not None:
                # ArUco检测
                aruco_result = detector.visualize_detection(color_image, depth_image, 'aruco')
                cv2.imshow('ArUco Detection', aruco_result) 
                cv2.waitKey(1)
                # 颜色检测
                #color_result = detector.visualize_detection(color_image, depth_image, 'color', color='red')
                #cv2.imshow('Color Detection', color_result)   
                #cv2.waitKey(1)    
                # 检测目标位姿
                target_pose = detector.get_target_pose_for_kinematics('aruco')
                if target_pose is not None:
                    print("[SUCCESS] 检测到目标位姿（相对于相机）:")
                    print(target_pose)
                else:
                    print("[ERROR] 没有检测到目标位姿")
                    print("[ERROR] 本次采样失败")
                    print(f"[INFO]: 累计已采样 {count} 次")
                    input("请按 Enter 键继续...")
                    continue


            # 将一组样本（robot_pose,target_pose）先转化为numpy格式再添加到calibrator
            robot_pose = np.array(robot_pose)
            target_pose = np.array(target_pose)
            calibrator.add_hand_eye_sample(robot_pose,target_pose)
            
            print(f"[SUCCESS]: 第 {i+1} / {len(joint_angles_list)}次采样成功。")
            count = count + 1
            print(f"[INFO]: 累计已采样 {count} 次")
            input("请按 Enter 键继续...")

        # 执行手眼标定
        calibrator.calibrate_hand_eye()
        # 保存手眼标定结果
        calibrator.save_hand_eye_result("hand_eye_calibration.json")
    
    finally:
        camera.stop_streaming()
        cv2.destroyAllWindows()
        
    
    print("=== 手眼标定完成 ===")

if __name__ == "__main__":
    main()
