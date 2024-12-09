#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  8 16:44:07 2024

@author: obum

11/8/24 Update: Changing from RPi.GPIO to pigpio to handle encoder ticks. RPi.GPIO struggled
with event detect. gpizero has no capability for event detection. 
"""

import pigpio
import time
import csv
import threading
from datetime import datetime
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    exit()

# I2C and ADC Setup
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS.ADS1115(i2c)
adc.gain = 1
chan = AnalogIn(adc, ADS.P0)

# ACS723 Settings
sensorSensitivity = 50 / 43.8  # Sensitivity in volts per amp (adjusted for your ACS723)
currentOffset = 2.5  # Offset voltage for zero current (2.5V for ACS723)

# Motor and Encoder Pins
M1 = 22       # Direction control pin
PWM_pin = 23  # PWM control pin
encoderPinA = 17  # Encoder Channel A pin
encoderPinB = 18  # Encoder Channel B pin

# Motor and Encoder Settings
PRESET_DISTANCE = 0.2
gearRatio = 986.41
countsPerMotorRevolution = 12
effectiveCountsPerOutputRevolution = countsPerMotorRevolution * gearRatio
motorShaftRadius = 0.009 # in meters

# Initialize encoder position and lock
encoder_position = 0
last_encoded = 0
encoder_lock = threading.Lock()

# Data log array
data_log = []

# PWM Setup using pigpio
pi.set_mode(M1, pigpio.OUTPUT)
pi.set_mode(PWM_pin, pigpio.OUTPUT)
pi.set_PWM_frequency(PWM_pin, 1000)  # 1kHz frequency for PWM

# Callback function for encoder updates
def update_encoder(gpio, level, tick):
    global encoder_position, last_encoded

    # Read the current states of encoder channels
    MSB = pi.read(encoderPinA)  # Read Channel A
    LSB = pi.read(encoderPinB)  # Read Channel B

    # Create a combined encoded state from MSB and LSB
    encoded = (MSB << 1) | LSB  # Combine A and B to a 2-bit value
    sum = (last_encoded << 2) | encoded  # Combine last and current states to detect transition

    with encoder_lock:
        if sum in [0b1101, 0b0100, 0b0010, 0b1011]:  # Clockwise movement
            encoder_position += 1
        elif sum in [0b1110, 0b0111, 0b0001, 0b1000]:  # Counter-clockwise movement
            encoder_position -= 1
        
        last_encoded = encoded

    # print("Encoder Position:", encoder_position)  # Debugging output

# Set up encoder pins as inputs with pull-up resistors
pi.set_mode(encoderPinA, pigpio.INPUT)
pi.set_mode(encoderPinB, pigpio.INPUT)
pi.set_pull_up_down(encoderPinA, pigpio.PUD_UP)
pi.set_pull_up_down(encoderPinB, pigpio.PUD_UP)

# Attach callback to encoderPinA for both rising and falling edges
pi.callback(encoderPinA, pigpio.EITHER_EDGE, update_encoder)
pi.callback(encoderPinB, pigpio.EITHER_EDGE, update_encoder)

# Function to move motor
def move_motor(speed, direction):
    pi.write(M1, direction)
    pi.set_PWM_dutycycle(PWM_pin, int(speed * 2.55))  # Convert speed to 0-255 scale

# Function to stop motor
def stop_motor():
    pi.set_PWM_dutycycle(PWM_pin, 0)

# Function to read current from ADS1115
def read_current():
    voltage = chan.voltage
    current = (voltage - currentOffset) / sensorSensitivity
    return current

# Function to log data
def log_data():
    global data_log
    timestamp = time.time() * 1000  # Current time in milliseconds

    with encoder_lock:
        position = encoder_position

    distance = (position / effectiveCountsPerOutputRevolution) * (2 * 3.14159 * motorShaftRadius)
    current = read_current()
    data_log.append([timestamp, current, position, distance])

# Main loop 1123
try:
    start_time = time.time()
    while time.time() - start_time < 15:  # Run for 15 seconds
        move_motor(40, 1)  # Example: move backward at 40% speed. 0 = R, 1 = L
        time.sleep(0.1)
        log_data()

        # Check if motor has reached the preset distance
        with encoder_lock:
            current_position = encoder_position
        if current_position >= PRESET_DISTANCE * effectiveCountsPerOutputRevolution / (2 * 3.14159 * motorShaftRadius):
            stop_motor()
            break

    # Save data to CSV at the end of the run
    with open('8_5_Slot_Up_1.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Timestamp (ms)', 'Current (A)', 'Position (counts)', 'Distance (m)'])
        writer.writerows(data_log)

finally:
    stop_motor()
    pi.stop()  # Stop pigpio
