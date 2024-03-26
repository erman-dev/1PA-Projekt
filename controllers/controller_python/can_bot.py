from controller import Robot, DistanceSensor
from math import sqrt
from math import cos, sin
import numpy as np


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
        steps_to_travel = n_steps
        direction = np.sign(n_steps)
        robot_bearing = [0, 1]  #self.get_bearing
        ir_prev = 1000
        on_line = 0
        off_line = 0
        
        # Variables for collision avoidance
        nr_measurements = 0
        d_front_bot_prev = 1000
        robot_passing = 0
        d_diff_threshold = 300 # Distance sifference threshold
        d_dist_threshold = 200 # Threshold distance
        stop_counter = 0
        ir_trend = 0 

        # Go!
        self.motor_left.setVelocity(self.speed_default * direction)
        self.motor_right.setVelocity(self.speed_default * direction)
       
        while self.time_step != -1:
            print("Hello World!")
            ir_left = self.infra_left.getValue()
            ir_right = self.infra_right.getValue()
            
            ir_rep = (ir_left + ir_right)/2
            nr_measurements = nr_measurements + 1

            if (abs(ir_rep - ir_prev) > 1):
                ir_trend = np.sign(ir_rep - ir_prev)

            # Prevent colision with enemy robot
            d_front_bot = self.dst_front_bot.getValue()
            d_front_diff = d_front_bot_prev - d_front_bot 

            # Robot detected passing in front of us
            if (d_front_diff > d_diff_threshold):
                robot_passing = 1
            elif (d_front_bot < d_dist_threshold and bool(robot_passing)):
                success = 0
                return
            
            # If the enemy robot is passing in front of us and we come close - stop
            if (bool(robot_passing) and d_front_bot < d_dist_threshold * 2):
                self.motor_left.setVelocity(0)
                self.motor_right.setVelocity(0)

                while (self.time_step(self.timeStep) != -1 and d_front_bot != 1000):
                    d_front_bot = self.dst_front_bot.getValue()
                    stop_counter += 1

                    if (stop_counter > 50):
                        success = 0
                        return

                self.motor_left.setVelocity(self.speed_default * direction)
                self.motor_right.setVelocity(self.speed_default * direction)   

                robot_passing = 0

            if (ir_trend == 1):
                on_line = 1
            elif ((ir_trend == 0 or ir_trend == -1) and bool(on_line)):
                off_line = 1
            
            # Ignore if the robot detects a line in the first few measurements
            # after start and after every line
            if (bool(on_line) and bool(off_line) and nr_measurements < 80/self.speed_default):
                on_line = 0
                off_line = 0
            elif (bool(on_line) and bool(off_line)):
                self.position = self.position + robot_bearing * direction
                steps_to_travel = steps_to_travel - np.sign(steps_to_travel)
                on_line = 0
                off_line = 0
                nr_measurements = 0

            # Destination reached
            if (steps_to_travel == 0):
                self.motor_left.setVelocity(0)
                self.motor_right.setVelocity(0)
                break

            ir_prev = ir_rep
            d_front_bot_prev = d_front_bot

            success = 1


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
        # Turn the robot to angle 1 of the scan
        self.turn(self.scan_angle[0])
        
        # Start the scan
        # Turn the robot to angle 2 of the scan
        scan_data = []
        self.motor_left.setVelocity(self.speed_default)
        self.motor_right.setVelocity(-self.speed_default)
        
        while(self.compass.getValue() < self.scan_angle[1]):
            # Get the distance to the can and the angle
            distance_can = self.dst_front_can.getValue()
            distance_bot = self.dst_front_bot.getValue()
            angle = self.compass.getValue()

            # Skip if the distance is too large
            if distance_can != 1000:
                # If the distance to the can is significantly different from the previous distance, save the data
                if abs(distance_can - prev_distance) > 50:
                    # Only save the data if the can is not the robot
                    if abs(distance_can - distance_bot) > 200:
                        scan_data.append([distance_can, angle])
                
            prev_distance = distance_can
        
        # Stop the scan, return to the default orientation
        self.turn(self.default_alignment)
        
        # Convert the distance and angle data to coordinates
        cans = []
        for distance, angle in scan_data:
            # Calculate the x and y coordinates based on distance and angle
            x = distance * cos(angle)
            y = distance * sin(angle) - 30
            
            # Round the coordinates to the nearest grid point
            x_grid = round(x / 200)
            y_grid = round(y / 200)

            # Skip if the position error is larger than 50
            if (x % 200 > 50) or (y % 200 > 50):
                continue
            
            # Add the coordinates to the list of cans
            cans.append([x_grid, y_grid])

        # Return the three nearest cans
        cans_sorted = sorted(cans, key=lambda pos: (abs(pos[0] - self.position[0]) + abs(pos[1] - self.position[1])))
        return cans_sorted[:min(3, len(cans_sorted))]