# MoveBase
A simple four Mecanum wheel control project,support up/back/left/right/rotation/45 degree oblique motion，The host using UART to send control command.  
control.py show how to control the motor
# IDE 
ST CubeIDE
# Chip
smt32f030F4P6
# Connection
Motor Driver chip:L298N  
Motor Index: (alse see in  control.py)  
0:right,front  
1:left,front  
2:right,back  
3:left,back  
            
Motor0:  
IN1->PA4  
IN2->PA5  
EN->PB1  
  
Motor1:  
IN1->PA0  
IN2->PA1  
EN->PB1  
    
Motor2:  
IN1->PA6  
IN2->PA7  
EN->PB1  
  
Motor3:  
IN1->PA9  
IN2->PA10  
EN->PB1  
