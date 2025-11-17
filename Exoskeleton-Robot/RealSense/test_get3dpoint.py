from realsense_camera import RealSenseCamera

import numpy as np

def test_get3dpoint():
    print("=== 根据像素坐标和深度值计算3D坐标 ===")
    
    # 创建相机对象
    camera = RealSenseCamera()

    # 加载相机内参
    camera.load_camera_info("camera_info.json")

    # 读取深度图原始数据
    depth_image = np.load("test_capture_depth.npy")

    # 计算像素点 (320,240) 对应的3D坐标
    pt3d = camera.get_3d_point(320, 240, depth_image)

    # 打印结果
    if pt3d is not None:
        X, Y, Z = pt3d
        print(f"像素 (320,120) → 3D坐标: X={X:.4f} m, Y={Y:.4f} m, Z={Z:.4f} m")
    else:
        print("该像素点深度无效或未获取到3D坐标。")
        
    print("=== 测试完成 ===")

if __name__ == "__main__":
    test_get3dpoint()