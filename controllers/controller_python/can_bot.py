from controller import Robot, DistanceSensor
from math import sqrt
from math import cos, sin

class Canbot:
    def __init__(self, 
                 robot: Robot,
                 motor_left: str, 
                 motor_right: str,
                 dst_front_can: str,
                 dst_front_bot: str,
                 compass: str,
                 infra_left: str,
                 infra_right: str,
                 time_step,
                 turn_slow_down_angle,
                 turn_angle_precision,
                 speed_default,
                 storage_positions,
                 position, 
                 default_alignment, 
                 scan_angle):
        # Save the robot and devices
        self.motor_left = robot.getDevice(motor_left)
        self.motor_right = robot.getDevice(motor_right)
        self.dst_front_can = robot.getDevice(dst_front_can)
        self.dst_front_bot = robot.getDevice(dst_front_bot)
        self.compass = robot.getDevice(compass)
        self.infra_left = robot.getDevice(infra_left)
        self.infra_right = robot.getDevice(infra_right)
        self.time_step = time_step
        self.storage_positions = storage_positions
        self.turn_slow_down_angle = turn_slow_down_angle
        self.turn_angle_precision = turn_angle_precision
        self.speed_default = speed_default
        self.position = position
        self.default_alignment = default_alignment
        self.scan_angle = scan_angle

        # Enable the sensors    
        self.dst_front_can.enable(self.time_step)
        self.dst_front_bot.enable(self.time_step)
        self.compass.enable(self.time_step)
        self.infra_left.enable(self.time_step)
        self.infra_right.enable(self.time_step)

        # Set the motor speeds at 0
        self.motor_left.setPosition(float('inf'))
        self.motor_right.setPosition(float('inf'))
        self.motor_left.setVelocity(0.0)
        self.motor_right.setVelocity(0.0)

    def go_to_position(self, position):
        """
        Go to the specified position.
        Position is an array of coordinates [x, y].
        """
        pass

    def travel(self, n_steps):
        """ Go speicified number of steps forwards of backwards."""
        pass

    def turn(self, angle):
        """ Turn the robot to a specified angle."""
        pass

    def store_cans(self):
        """ Calculate the path to the storage.
        Go to the storage.
        Turn si the robot can back out.
        Dump the cans by backing out.
        Turn to the default orinentation."""
        pass

    def scan_cans(self):
        """
        Return coordinates of the three nearest cans."""
        pass