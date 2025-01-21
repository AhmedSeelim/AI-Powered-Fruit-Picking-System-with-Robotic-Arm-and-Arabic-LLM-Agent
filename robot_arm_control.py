import RPi.GPIO as GPIO
import time
import math
import numpy as np
import atexit

class RobotArm6DOF:
    def __init__(self):
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Define servo pins
        self.SERVO_PINS = {
            'base': 17,      # Base rotation
            'shoulder': 18,  # Shoulder joint
            'elbow': 27,     # Elbow joint
            'wrist_pitch': 22,  # Wrist up/down
            'wrist_roll': 23,   # Wrist rotation
            'gripper': 24    # Gripper
        }

        # Initial angles (degrees)
        self.INIT_ANGLES = {
            'base': 90,      # Center
            'shoulder': 125, # 45 degrees up
            'elbow': 135,    # 60 degrees
            'wrist_pitch': 45,  # Level
            'wrist_roll': 90,   # Center
            'gripper': 170    # Half open
        }

        # Servo parameters
        self.SERVO_PARAMS = {
            'base': {'min_angle': 0, 'max_angle': 250, 'min_duty': 2, 'max_duty': 12},      # S3003
            'shoulder': {'min_angle': 0, 'max_angle': 250, 'min_duty': 2, 'max_duty': 12},  # MG995
            'elbow': {'min_angle': 0, 'max_angle': 250, 'min_duty': 2, 'max_duty': 12},     # MG995
            'wrist_pitch': {'min_angle': 0, 'max_angle': 180, 'min_duty': 2.5, 'max_duty': 12.5}, # SG90
            'wrist_roll': {'min_angle': 0, 'max_angle': 180, 'min_duty': 2.5, 'max_duty': 12.5},  # SG90
            'gripper': {'min_angle': 0, 'max_angle': 180, 'min_duty': 2.5, 'max_duty': 12.5}      # SG90
        }

        # Link lengths (in cm)
        self.base_height = 5
        self.shoulder_len = 3
        self.l1 = 12  # Upper arm
        self.l2 = 9   # Forearm
        self.wrist_len = 3
        self.gripper_len = 4

        # Initialize servos
        self.servos = {}
        self.setup_gpio()

        # Register cleanup
        atexit.register(self.cleanup)

    def setup_gpio(self):
        """Set up GPIO pins for all servos"""
        for name, pin in self.SERVO_PINS.items():
            GPIO.setup(pin, GPIO.OUT)
            self.servos[name] = GPIO.PWM(pin, 50)  # 50Hz frequency
            self.servos[name].start(0)

    def angle_to_duty_cycle(self, servo_name, angle):
        """Convert angle to duty cycle based on servo type"""
        params = self.SERVO_PARAMS[servo_name]
        angle = np.clip(angle, params['min_angle'], params['max_angle'])

        # Linear interpolation
        duty = (angle - params['min_angle']) / (params['max_angle'] - params['min_angle'])
        duty = duty * (params['max_duty'] - params['min_duty']) + params['min_duty']
        return duty

    def move_to_initial_position(self):
        """Move all servos to their initial positions"""
        print("Moving to initial position...")

        # Move servos one by one from base to tip
        servo_order = ['base', 'shoulder', 'elbow', 'wrist_pitch', 'wrist_roll', 'gripper']

        for name in servo_order:
            print(f"Initializing {name} to {self.INIT_ANGLES[name]}°")
            duty = self.angle_to_duty_cycle(name, self.INIT_ANGLES[name])
            self.servos[name].ChangeDutyCycle(duty)
            time.sleep(0.5)  # Longer delay for initialization
            self.servos[name].ChangeDutyCycle(0)

        print("Initial position reached")
        time.sleep(1)  # Wait for stability

    def move_servo(self, servo_name, angle):
        """Move a specific servo to the given angle"""
        try:
            duty = self.angle_to_duty_cycle(servo_name, angle)
            self.servos[servo_name].ChangeDutyCycle(duty)
            time.sleep(0.3)  # Allow servo to reach position
            self.servos[servo_name].ChangeDutyCycle(0)  # Stop PWM
        except Exception as e:
            print(f"Error moving {servo_name} servo: {e}")

    def inverse_kinematics(self, x, y, z):
        """Calculate joint angles for given end effector position"""
        try:
            # Calculate base angle
            theta0 = math.degrees(math.atan2(y, x))
            theta0 = np.interp(theta0, [-90, 90], [0, 250])  # Map to servo range

            # Calculate distance to target
            r = math.sqrt(x*x + y*y)

            # Adjust target for end effector length
            r -= (self.wrist_len + self.gripper_len)
            z -= self.base_height

            # Calculate planar reach
            D = math.sqrt(r*r + z*z)

            if D > (self.l1 + self.l2):
                raise ValueError("Target out of reach")

            # Calculate elbow angle using cosine law
            cos_theta2 = (D*D - self.l1*self.l1 - self.l2*self.l2) / (2 * self.l1 * self.l2)
            theta2 = math.degrees(math.acos(np.clip(cos_theta2, -1, 1)))

            # Calculate shoulder angle
            theta1 = math.degrees(math.atan2(z, r) +
                                math.atan2(self.l2 * math.sin(math.radians(theta2)),
                                         self.l1 + self.l2 * math.cos(math.radians(theta2))))

            # Map shoulder and elbow angles to servo ranges (0-250)
            theta1 = np.interp(theta1, [0, 180], [0, 250])
            theta2 = np.interp(theta2, [0, 180], [0, 250])

            # Calculate wrist angle to keep end effector level
            theta3 = theta1 + theta2 - 180
            theta3 = np.clip(theta3, 0, 180)  # Map to SG90 range

            return {
                'base': theta0,
                'shoulder': theta1,
                'elbow': theta2,
                'wrist_pitch': theta3,
                'wrist_roll': 90  # Keep centered
            }

        except Exception as e:
            print(f"Inverse kinematics error: {e}")
            return None

    def move_to_target(self, x, y, z):
        """Move end effector to target position"""
        angles = self.inverse_kinematics(x, y, z)

        if angles:
            print(f"\nMoving to target position ({x}, {y}, {z}) cm")
            print(f"Calculated angles: Base={angles['base']:.1f}°, "
                  f"Shoulder={angles['shoulder']:.1f}°, "
                  f"Elbow={angles['elbow']:.1f}°, "
                  f"Wrist Pitch={angles['wrist_pitch']:.1f}°, "
                  f"Wrist Roll={angles['wrist_roll']:.1f}°")

            # Move servos in sequence from base to tip
            for servo_name, angle in angles.items():
                self.move_servo(servo_name, angle)
                time.sleep(5)

            print("Target position reached")
            return True
        return False

    def cleanup(self):
        """Clean up GPIO and stop PWM"""
        try:
            for servo in self.servos.values():
                servo.ChangeDutyCycle(0)
                time.sleep(0.1)
                servo.stop()
            GPIO.cleanup()
        except Exception as e:
            print(f"Cleanup error: {e}")

def main():
    try:
        # Create robot arm instance
        arm = RobotArm6DOF()

        # First move to initial position
        arm.move_to_initial_position()

        # Wait for stability
        time.sleep(5)

        # Then move to target using IK
        x, y, z = 10.0, 0.0, 15.0
        arm.move_to_target(x, y, z)

    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
