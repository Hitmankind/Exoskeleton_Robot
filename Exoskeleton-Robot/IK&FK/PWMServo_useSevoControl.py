#!/usr/bin/python3
# coding=utf8
import sys
import time
import ServoControl as ServoControl

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
if __name__ == '__main__':
    #舵机偏差，每个机械爪偏差都不同，需要根据自己的机械爪情况修改，偏差范围-100 ~ 100。
    deviation = 0
    while(1):
        #pwm舵机无法给舵机限位，所以在程序中设置舵机脉冲宽度不能超过安全范围。
        #本款机械爪脉冲宽度范围 500 ~ 1700，超过这个范围可能会造成舵机损坏。
        # 参数：参数1：舵机接口编号; 参数2：位置; 参数3：运行时间
        ServoControl.setPWMServoMove(3, 1500 + deviation, 500) # 1号pwm舵机转到1500位置，用时500ms。机械爪舵机回到中位。
        time.sleep(2) # 延时时间
        
        ServoControl.setPWMServoMove(3, 500 + deviation, 500) #爪子为最大角度张开
        time.sleep(2)
        
        ServoControl.setPWMServoMove(3, 1700 + deviation, 500)   #爪子为闭合。爪子长时间闭合夹紧物品，会使舵机温度升高，从而造成堵转风险，请根据实际物品情况进行适当调整。
        time.sleep(2)












