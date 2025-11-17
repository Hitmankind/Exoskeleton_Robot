from realsense_camera import RealSenseCamera

import time

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

        # （可选）从文件加载相机内参并打印
        camera.load_camera_info("camera_info.json")
        print(f"已加载相机内参: {camera.intrinsics}")
        
        # 捕获几帧图像
        for i in range(5):
            color_image, depth_image = camera.get_frames()
            
            if color_image is not None and depth_image is not None:
                print(f"第{i+1}帧: 彩色图 {color_image.shape}, 深度图 {depth_image.shape}")
                
                # 显示图像（可选）
                #cv2.imshow('Color Image', color_image)
                
                # 深度图可视化
                #depth_colormap = cv2.applyColorMap(
                #    cv2.convertScaleAbs(depth_image, alpha=0.03), 
                #    cv2.COLORMAP_JET
                #)
                #cv2.imshow('Depth Image', depth_colormap)
                
                # 按ESC退出
                #if cv2.waitKey(1) & 0xFF == 27:
                #    break
            else:
                print(f"第{i+1}帧获取失败")
        
        # 保存相机信息
        #camera.save_camera_info("camera_info.json")
        
        # 捕获测试图像
        camera.capture_image("test_capture")
        
    finally:
        # 清理
        camera.stop_streaming()
        #cv2.destroyAllWindows()
    
    print("=== 测试完成 ===")

if __name__ == "__main__":
    test_camera()