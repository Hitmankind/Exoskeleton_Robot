import ServoControl
import time 

if __name__ == '__main__': 
     #while True:
        #ServoControl.setPWMServoMove(0,500,2000)
        for pwm in range(500,2501,10):
            ServoControl.setPWMServoMove(23, pwm, 1500)
            time.sleep(1)
            print(pwm)
       # ServoControl.setPWMServoAngle(23, 120, 2000)
        #time.sleep(10)
        ServoControl.setPWMServoAngle(1, 90, 2000)
        #time.sleep(10)
        #ServoControl.setPWMServoAngle(0,90,1000)
        #time.sleep(10)
        #ServoControl.setPWMServoAngle(2,135,2000)
        #time.sleep(10)
        #ServoControl.setPWMServoAngle(2,180,1000)



