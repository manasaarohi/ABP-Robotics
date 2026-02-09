#!/usr/bin/env python3
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, MoveTank, SpeedPercent, LargeMotor, MediumMotor, Motor
import time
import sys
import threading
import tty
import termios
import math

# --- CONFIGURATION ---
tank = MoveTank(OUTPUT_A, OUTPUT_B)
speed = 30
TURN_MULTIPLIER = 2.2
WHEEL_DIAMETER_CM = 5.5
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_CM * math.pi

#  ARM & CLAW SETUP 
try:
    # "Motor" is generic and works for both EV3 (White) and NXT (Orange) motors
    # Ensure Arm is in Port C and Claw is in Port D
    arm = Motor(OUTPUT_C) 
    claw = Motor(OUTPUT_D)
    
    # THE GRAVITY FIX:
    # 'hold' forces the motor to use power to stay in place when stopped.
    arm.stop_action = 'hold' 
    claw.stop_action = 'hold'
except Exception as e:
    print("WARNING: Arm or Claw motors not found. Check Ports C and D.")
    print("Details: {}".format(e))
    arm = None
    claw = None

tank.left_motor.ramp_up_sp = 400
tank.right_motor.ramp_up_sp = 400
tank.left_motor.ramp_down_sp = 400
tank.right_motor.ramp_down_sp = 400

STOP_REQUESTED = False
old_settings = None 

def keyboard_monitor():
    global STOP_REQUESTED
    try:
        tty.setraw(sys.stdin.fileno())
        while not STOP_REQUESTED:
            char = sys.stdin.read(1)
            # Check for CTRL+C (x03) or '-' key
            if char == '-' or char == '\x03': 
                STOP_REQUESTED = True
                break
    except:
        pass

def wait_and_check():
    # Helper to pause while motors run, but keep checking for STOP
    while (tank.left_motor.is_running or 
           tank.right_motor.is_running or 
           (arm and arm.is_running) or 
           (claw and claw.is_running)):
        
        if STOP_REQUESTED:
            tank.off(brake=True)
            if arm: arm.stop()
            if claw: claw.stop()
            sys.exit(0)
        time.sleep(0.01)

#  MOVEMENT FUNCTIONS

def forward(cm):
    if STOP_REQUESTED: return
    # This logic converts CENTIMETERS to Rotations.
    rotations = cm / WHEEL_CIRCUMFERENCE
    tank.on_for_rotations(SpeedPercent(speed), SpeedPercent(speed), rotations, brake=True, block=False)
    wait_and_check()

def backward(cm):
    if STOP_REQUESTED: return
    rotations = cm / WHEEL_CIRCUMFERENCE
    # Negative speed moves the robot backward
    tank.on_for_rotations(SpeedPercent(-speed), SpeedPercent(-speed), rotations, brake=True, block=False)
    wait_and_check()

def left(degrees):
    if STOP_REQUESTED: return
    wheel_turn = degrees * TURN_MULTIPLIER
    tank.on_for_degrees(SpeedPercent(-speed), SpeedPercent(speed), wheel_turn, brake=True, block=False)
    wait_and_check()

def right(degrees):
    if STOP_REQUESTED: return
    wheel_turn = degrees * TURN_MULTIPLIER
    tank.on_for_degrees(SpeedPercent(speed), SpeedPercent(-speed), wheel_turn, brake=True, block=False)
    wait_and_check()

# NEW ARM FUNCTIONS

def lower_arm(degrees=90, speed_sp=30):
    if STOP_REQUESTED or not arm: return
    arm.on_for_degrees(SpeedPercent(-speed_sp), degrees, block=False)
    wait_and_check()

def lift_arm(degrees=90, speed_sp=20):
    if STOP_REQUESTED or not arm: return
    # Use negative speed to go down
    arm.on_for_degrees(SpeedPercent(speed_sp), degrees, block=False)
    wait_and_check()

def grab():
    if STOP_REQUESTED or not claw: return
    claw.on(SpeedPercent(50), block=False)
    time.sleep(1.0) 
    claw.stop()
    
def release():
    if STOP_REQUESTED or not claw: return
    claw.on_for_degrees(SpeedPercent(-50), 100, block=False)
    wait_and_check()

# INSTRUCTIONS
instructions = [
    lambda: lift_arm(30),
    lambda: forward(30),
    lambda: grab(),
    lambda: backward(45),
    lambda: release()
]

if __name__ == "__main__":
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    listener_thread = threading.Thread(target=keyboard_monitor, daemon=True)
    listener_thread.start()

    try:
        for step in instructions:
            if STOP_REQUESTED: break
            step()
            time.sleep(0.5)

    except Exception as e:
        print("\r\nError: {}".format(e))
        
    finally:
        tank.off(brake=True)
        if arm: arm.stop()
        if claw: claw.stop()
        if old_settings:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
