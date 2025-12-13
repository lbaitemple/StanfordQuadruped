#!/usr/bin/env python
# coding: utf-8
#
# Tug-of-War Touch Control for Mini Pupper
# - Back sensor: Start tug-of-war (pulling)
# - Front sensor: Stop tug-of-war
# - Left sensor: Reduce pull strength
# - Right sensor: Hold stance without pulling
#

import numpy as np
import time
import RPi.GPIO as GPIO

from src.IMU import IMU
from src.Controller import Controller
from src.State import State
from src.Command import Command
from src.MovementGroup import MovementGroups
from src.MovementScheme import MovementScheme
from MangDang.mini_pupper.HardwareInterface import HardwareInterface
from MangDang.mini_pupper.Config import Configuration
from MangDang.mini_pupper.display import Display
from pupper.Kinematics import four_legs_inverse_kinematics

# GPIO pin definitions (BCM mode)
TOUCH_FRONT = 6
TOUCH_BACK = 2
TOUCH_LEFT = 3
TOUCH_RIGHT = 16

# Debounce time in seconds
DEBOUNCE_TIME = 0.3


def setup_gpio():
    """Initialize GPIO pins for touch sensors."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(TOUCH_FRONT, GPIO.IN)
    GPIO.setup(TOUCH_BACK, GPIO.IN)
    GPIO.setup(TOUCH_LEFT, GPIO.IN)
    GPIO.setup(TOUCH_RIGHT, GPIO.IN)


def is_touched(pin):
    """Check if a touch sensor is activated (LOW = touched)."""
    return not GPIO.input(pin)


def main(use_imu=False):
    """Main control loop for tug-of-war with touch sensors."""
    print("Tug-of-War Touch Control")
    print("========================")
    print("Back sensor  -> Start pulling")
    print("Front sensor -> Stop and return to stance")
    print("Left sensor  -> Reduce pull strength to 50%")
    print("Right sensor -> Hold stance (stop pulling)")
    print("Ctrl+C to exit")
    print()

    # Initialize GPIO
    setup_gpio()

    # Initialize robot hardware
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create IMU handle if needed
    if use_imu:
        imu = IMU(port="/dev/ttyACM0")
        imu.flush_buffer()

    # Create controller
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    # Initialize MovementGroups with default stop stance
    Move = MovementGroups()
    Move.stop(time=0.5)  # Start with default stance
    
    # Create movement scheme
    movementCtl = MovementScheme(Move.MovementLib)

    # Command setup
    command = Command()
    command.pseudo_dance_event = True

    # Control state
    tug_active = False
    last_trigger_time = 0
    last_loop = time.time()

    try:
        while True:
            now = time.time()
            if now - last_loop < config.dt:
                continue
            last_loop = now

            current_time = time.time()

            # Read IMU data
            quat_orientation = (
                imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
            )
            state.quat_orientation = quat_orientation

            # Check touch sensors
            back_touched = is_touched(TOUCH_BACK)
            front_touched = is_touched(TOUCH_FRONT)
            left_touched = is_touched(TOUCH_LEFT)
            right_touched = is_touched(TOUCH_RIGHT)

            # Back sensor - start tug-of-war
            if back_touched and (current_time - last_trigger_time > DEBOUNCE_TIME):
                if not tug_active:
                    print("[BACK] Starting tug-of-war - PULLING (full strength)")
                    Move.MovementLib.clear()
                    Move.tug_of_war(pulling=True, pull_strength=1.0)
                    movementCtl = MovementScheme(Move.MovementLib)
                    tug_active = True
                    print(f"[DEBUG] Movement created, velocity should be: -0.15 m/s")
                last_trigger_time = current_time

            # Front sensor - stop tug-of-war
            elif front_touched and (current_time - last_trigger_time > DEBOUNCE_TIME):
                print("[FRONT] Stopping tug-of-war")
                Move.MovementLib.clear()
                Move.stop(time=0.5)
                movementCtl = MovementScheme(Move.MovementLib)
                tug_active = False
                last_trigger_time = current_time

            # Left sensor - reduce strength
            elif left_touched and tug_active and (current_time - last_trigger_time > DEBOUNCE_TIME):
                print("[LEFT] Reducing pull strength to 50%")
                Move.MovementLib.clear()
                Move.tug_of_war(pulling=True, pull_strength=0.5)
                movementCtl = MovementScheme(Move.MovementLib)
                last_trigger_time = current_time

            # Right sensor - hold stance
            elif right_touched and tug_active and (current_time - last_trigger_time > DEBOUNCE_TIME):
                print("[RIGHT] Hold stance (stop pulling, stay braced)")
                Move.MovementLib.clear()
                Move.tug_of_war(pulling=False, pull_strength=1.0)
                movementCtl = MovementScheme(Move.MovementLib)
                last_trigger_time = current_time

            # Run movement scheme and update command
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()

            # Debug: print velocity every 2 seconds when active
            if tug_active and int(current_time * 0.5) != int((current_time - config.dt) * 0.5):
                print(f"[DEBUG] velocity: {command.horizontal_velocity}, pitch: {command.pitch:.2f}")

            # Run controller and update hardware
            controller.run(state, command, disp)
            hardware_interface.set_actuator_postions(state.joint_angles)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Return to default stance before cleanup
        Move.MovementLib.clear()
        Move.stop(time=0.3)
        movementCtl = MovementScheme(Move.MovementLib)
        
        # Run a few cycles to return to stance
        for _ in range(50):
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)
            hardware_interface.set_actuator_postions(state.joint_angles)
            time.sleep(config.dt)

        GPIO.cleanup()
        print("GPIO cleaned up. Goodbye!")


if __name__ == "__main__":
    main(use_imu=False)