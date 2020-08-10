#!/usr/bin/python3

import serial
import _thread
import sys
import tty
import termios
import traceback

_serail=None

def openSerial():
    global _serail
    port="/dev/ttyUSB0"
    baundrate=38400
    _serail=serial.Serial(port,baundrate)

def writeCmd(cmd):
    _serail.write(cmd.encode("utf-8"))

def reciveLoop():
    while True:
        s=_serail.readline()
        print(s.decode("utf-8"),end="")

def reciveInfo():
    _thread.start_new_thread(reciveLoop,())

#		   coordinate		
# 		X
# 		^
# 		|
# 		|
# 		|------->Y	
#
# 		Vetical View
# 		1--------0
#       |               |
# 		|               |	
# 		|               |
# 		3--------2
#
# 		Matrix
# 1->postive  -1->negtive
# X: 1,1,1,1
# Y: -1,1,1,-1
# Z: 1,-1,1,-1 
HEIGHT=0.2
WIDTH=0.2
RADIU=0.08
X_E=1/RADIU
Y_E=1/RADIU
Z_E=(HEIGHT+WIDTH)/RADIU
def setSpeed(vx,vy,wz):
    w0=X_E*vx-Y_E*vy+Z_E*wz
    w1=X_E*vx+Y_E*vy-Z_E*wz
    w2=X_E*vx+Y_E*vy+Z_E*wz
    w3=X_E*vx-Y_E*vy-Z_E*wz

    if abs(w0)!=abs(w1) or abs(w1)!=abs(w2) or abs(w2)!=abs(w3):
        print("do not support this speed(%f,%f,%f)"%(vx,vy,wz))
        return
    cmd="S"+str(int(abs(w0)))
    cmd=cmd+"D"+('0' if w0>0 else '1')
    cmd=cmd+"D"+('0' if w1>0 else '1')
    cmd=cmd+"D"+('0' if w2>0 else '1')
    cmd=cmd+"D"+('0' if w3>0 else '1')
    print("send:",cmd)
    writeCmd(cmd+"\r\n")
    
def stopMotor():
    writeCmd("OFF\n")

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def keyborad_ctl():
    speed=2.0
    try:
        while True:
            c=readchar()
            if c=='q':
                setSpeed(0,-speed,0)
            elif c=="e":
               setSpeed(0,speed,0)
            elif c== "w":
               setSpeed(speed,0,0)
            elif c=="s":
                setSpeed(-speed,0,0)
            elif c=="a":
                setSpeed(0,0,speed)
            elif c=="d":
               setSpeed(0,0,-speed)
            elif c=="\033":
                break
            else: 
                break
    except Exception as e:
        print(e)
        traceback.print_exc()
    stopMotor()

if __name__ == "__main__":
    openSerial()
    reciveInfo()
    keyborad_ctl()
    
