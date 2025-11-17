from roboticstoolbox import DHRobot, RevoluteDH
from ServoControl import setPWMServoAngle, pwmServoPositionToAngle
import numpy as np
import math
import time 
# 全局变量存储舵机状态（因为ServoControl.py没有getPWMServoAngle函数）
servo_angles = [90, 90, 65]  # 默认中位角度

# 舵机ID映射函数：将逻辑ID映射到物理ID
def map_servo_id(logical_id):
    """将逻辑舵机ID映射到物理舵机ID"""
    if logical_id == 0:
        return 23  # 0号口失效，0号舵机改用23号口
    else:
        return logical_id

# DH参数配置（其余参数作为可设置变量）
class ArmRobot(DHRobot):
    def __init__(self):
        links = [
            RevoluteDH(d=-0.0237, a=0.0435, alpha=-2.7611),  # 关节1
            RevoluteDH(d=0.0006, a=0.2884, alpha=1.5708),       # 关节2
            RevoluteDH(d=0.0004, a=0.1536, alpha=0)       # 关节3
        ]
        super().__init__(links, name='ExoskeletonArm')

# 设置舵机角度并更新全局状态
def setPWMServoAngleWithTracking(servo_id, angle, time=2000):
    """设置舵机角度并跟踪状态"""
    global servo_angles
    # 确保角度在0-180度范围内
    angle = max(0, min(180, angle))
    # 映射舵机ID
    physical_servo_id = map_servo_id(servo_id)
    # 设置舵机
    setPWMServoAngle(physical_servo_id, angle, time)
    # 更新全局状态（使用逻辑ID）
    if 0 <= servo_id < len(servo_angles):
        servo_angles[servo_id] = angle
    return angle

# 正运动学解算：给定关节角度，计算末端位置
def forward_kinematics(joint_angles_deg=None):
    """
    正运动学：根据关节角度计算末端位置
    参数:
        joint_angles_deg: 关节角度列表（度），如果为None则使用当前舵机角度
    返回:
        4x4变换矩阵表示末端位置和姿态
    """
    robot = ArmRobot()
    
    # 如果没有提供角度，使用当前舵机角度
    if joint_angles_deg is None:
        joint_angles_deg = servo_angles.copy()
    
    # 设置舵机到指定角度（使用逻辑ID）
    for i, angle_deg in enumerate(joint_angles_deg):
        setPWMServoAngleWithTracking(i, angle_deg)
    
    # 转换度数到弧度
    joint_angles_rad = [math.radians(angle) for angle in joint_angles_deg]
    
    # 计算正运动学
    end_effector_pose = robot.fkine(joint_angles_rad)
    
    print(f"关节角度: {joint_angles_deg}")
    print(f"末端位置: {end_effector_pose}")
    
    return end_effector_pose

# 逆运动学解算：给定末端位置，计算关节角度
def inverse_kinematics(target_pose):
    """
    逆运动学：根据目标末端位置计算关节角度
    参数:
        target_pose: 4x4变换矩阵表示目标位置和姿态
    返回:
        关节角度列表（度）
    """
    robot = ArmRobot()
    
    # 使用逆运动学求解
    solution = robot.ikine_LM(target_pose)
    
    if solution.success:
        joint_angles_rad = solution.q
        # 转换弧度到度数
        joint_angles_deg = [math.degrees(angle) for angle in joint_angles_rad]
        
        # 设置舵机到计算出的角度（使用逻辑ID）
        for i, angle_deg in enumerate(joint_angles_deg):
            # 确保角度在0-180度范围内
            angle_deg = max(0, min(180, angle_deg))
            setPWMServoAngleWithTracking(i, angle_deg)
        
        print(f"逆运动学求解成功")
        print(f"目标位置: {target_pose}")
        print(f"计算出的关节角度: {joint_angles_deg}")
        
        return joint_angles_deg
    else:
        print("警告：逆运动学求解失败")
        return None

# 初始化舵机到中位位置
def initialize_servos():
    """初始化所有舵机到中位位置（90度）"""
    print("初始化舵机到中位位置...")
    for i in range(3):
        setPWMServoAngleWithTracking(i, 90)
    print("舵机初始化完成")

# 测试函数
def test_kinematics():
    """测试正逆运动学功能"""
    print("=== 运动学测试 ===")
    
    
    # 初始化舵机
    initialize_servos()
    time.sleep(5)
    # 测试正运动学
    print("\n1. 测试正运动学（设置角度[45, 90, 135]）:")
    forward_kinematics([90,90,90])    
    # 测试逆运动学
    print("\n2. 测试逆运动学:")
    target = np.array([[0, 0, 1, 0.3],
                       [0, 1, 0, 0],
                       [-1, 0, 0, 0.2],
                       [0, 0, 0, 1]])
    result_angles = inverse_kinematics(target)
    
    if result_angles:
        print(f"逆运动学结果: {result_angles}")
    
    print("\n=== 测试完成 ===")

if __name__ == '__main__':
    # 运行测试
    test_kinematics()
    print("Now robot is ")
    forward_kinematics() 