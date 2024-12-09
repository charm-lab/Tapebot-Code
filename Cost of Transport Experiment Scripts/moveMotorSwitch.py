import pigpio
import time

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    exit()

# Motor pins
M1 = 22        # Direction control pin
PWM_pin = 23   # PWM control pin
switch_pin1 = 26  # GPIO pin for first direction (forward)
switch_pin2 = 20  # GPIO pin for second direction (reverse)

# Set motor speed as a variable (0-100 for percentage speed)
motor_speed = 40  # Set motor speed to 40% (can be adjusted)

# Set up motor pins
pi.set_mode(M1, pigpio.OUTPUT)
pi.set_mode(PWM_pin, pigpio.OUTPUT)
pi.set_PWM_frequency(PWM_pin, 1000)  # Set PWM frequency to 1kHz

# Set up switch pins
pi.set_mode(switch_pin1, pigpio.INPUT)
pi.set_mode(switch_pin2, pigpio.INPUT)
pi.set_pull_up_down(switch_pin1, pigpio.PUD_DOWN)  # Pull-down for switch pin 1
pi.set_pull_up_down(switch_pin2, pigpio.PUD_DOWN)  # Pull-down for switch pin 2

# Motor control function
def move_motor(speed, direction):
    pi.write(M1, direction)
    pi.set_PWM_dutycycle(PWM_pin, int(speed * 2.55))  # Convert speed to 0-255

# Main loop
try:
    while True:
        if pi.read(switch_pin1) == 1:  # If switch position 1 (GPIO 26) is HIGH green RIGHT
            move_motor(motor_speed, 1)  # Move forward at the specified speed
            print(f"Motor moving forward at {motor_speed}% speed")
        elif pi.read(switch_pin2) == 1:  # If switch position 2 (GPIO 20) is HIGH blue LEFT
            move_motor(motor_speed, 0)  # Move in reverse at the specified speed
            print(f"Motor moving reverse at {motor_speed}% speed")
        else:  # If switch is in the OFF position
            move_motor(0, 0)  # Stop motor
            print("Motor stopped")

        time.sleep(0.1)  # Delay for responsiveness

finally:
    pi.stop()
    print("Motor control stopped. Program exited.")
