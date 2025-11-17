import time
import serial
#ccnuSDK 因为24控制器方案所以删除了原有关总线舵机的控制
LOBOT__FRAME_HEADER              = 0x55
LOBOT_CMD_SERVO_MOVE             = 3
LOBOT_CMD_ACTION_GROUP_RUN       = 6
LOBOT_CMD_ACTION_GROUP_STOP      = 7
LOBOT_CMD_ACTION_GROUP_SPEED     = 11
LOBOT_CMD_GET_BATTERY_VOLTAGE    = 15
LOBOT_CMD_SERVO_POS_READ         = 28  # 读取舵机位置指令

serialHandle = serial.Serial("/dev/ttyAMA0", 9600)  # 初始化串口， 波特率为9600


#控制单个PWM舵机转动
def setPWMServoMove(servo_id, servo_pulse, time):
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(0x08) #数据长度
    buf.append(LOBOT_CMD_SERVO_MOVE) #指令
    buf.append(0x01) #要控制的舵机个数
    
    time = 0 if time < 0 else time
    time = 30000 if time > 30000 else time
    time_list = list(time.to_bytes(2, 'little'))    #时间
    buf.append(time_list[0])
    buf.append(time_list[1])    

    servo_id = 254 if (servo_id < 1 or servo_id > 254) else servo_id
    buf.append(servo_id) #舵机ID
    
    servo_pulse = 500 if servo_pulse < 500 else servo_pulse
    servo_pulse = 2500 if servo_pulse > 2500 else servo_pulse
    pulse_list = list(servo_pulse.to_bytes(2, 'little'))    #位置
    buf.append(pulse_list[0])
    buf.append(pulse_list[1])     

    serialHandle.write(buf)
    
#控制多个PWM舵机转动
def setPWMServoMoveByArray(servos, servos_count, time):
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(servos_count*3+5) #数据长度
    buf.append(LOBOT_CMD_SERVO_MOVE) #指令
    
    servos_count = 1 if servos_count < 1 else servos_count
    servos_count = 254 if servos_count > 254 else servos_count
    buf.append(servos_count) #要控制的舵机个数
    
    time = 0 if time < 0 else time
    time = 30000 if time > 30000 else time
    time_list = list(time.to_bytes(2, 'little'))
    buf.append(time_list[0])    #时间
    buf.append(time_list[1])
    
    for i in range(servos_count):
        buf.append(servos[i*2]) #舵机ID

        pos = servos[i*2+1]
        pos = 500 if pos < 500 else pos
        pos = 2500 if pos > 2500 else pos
        pos_list = list(pos.to_bytes(2, 'little'))
        buf.append(pos_list[0])    #位置
        buf.append(pos_list[1])

    serialHandle.write(buf)

def setGroupRun(group_id, group_count):
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(5) #数据长度
    buf.append(LOBOT_CMD_ACTION_GROUP_RUN) #指令
    buf.append(group_id)  #动作组id
    count_list = list(group_count.to_bytes(2, 'little'))
    buf.append(count_list[0])    #次数
    buf.append(count_list[1])
    
    serialHandle.write(buf)
    
def setGroupStop():
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(2) #数据长度
    buf.append(LOBOT_CMD_ACTION_GROUP_STOP) #指令
    serialHandle.write(buf)
    
def setGroupSpeed(group_id, group_speed):
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(5) #数据长度
    buf.append(LOBOT_CMD_ACTION_GROUP_SPEED) #指令
    buf.append(group_id)  #动作组id
    
    speed_list = list(group_speed.to_bytes(2, 'little'))
    buf.append(speed_list[0])    #速度
    buf.append(speed_list[1])
    serialHandle.write(buf)

# 计算校验和
def calculateChecksum(buf):
    checksum = 0
    for i in range(2, len(buf)):  # 从第3个字节开始计算（跳过帧头）
        checksum += buf[i]
    return (~checksum) & 0xFF


# 将PWM舵机位置值转换为角度
def pwmServoPositionToAngle(position):
    """
    将PWM舵机位置值转换为角度
    参数:
        position: 位置值 (500-2500)
    返回:
        角度值 (0-180度)
    """
    if position < 500 or position > 2500:
        return -1
    return ((position - 500) * 180.0) / 2000.0


#控制PWM舵机按角度转动
def setPWMServoAngle(servo_id, angle, time=2000):
    """
    控制PWM舵机按角度转动
    参数:
        servo_id: 舵机ID (0-23)
        angle: 目标角度 (0-180度)
        time: 运行时间 (毫秒，默认2000ms)
    """
    # 验证servo_id范围 (0-23)
    if servo_id < 0 or servo_id > 23:
        print(f"警告: servo_id {servo_id} 超出范围 (0-23)，已限制到有效范围")
        servo_id = max(0, min(23, servo_id))
    
    # 验证角度范围 (0-180度)
    if angle < 0 or angle > 180:
        print(f"警告: 角度 {angle} 超出范围 (0-180度)，已限制到有效范围")
        angle = max(0, min(180, angle))
    
    # 将角度转换为脉冲值 (0度对应500us，180度对应2500us)
    servo_pulse = int(500 + (angle / 180.0) * (2500 - 500))
    
    # 调用setPWMServoMove函数
    setPWMServoMove(servo_id, servo_pulse, time)
    
    return servo_pulse
    
