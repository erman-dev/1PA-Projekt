"""controller_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import Robot
from can_bot import Canbot
import numpy as np




print("Hello World!")




storage_positions = [[7, 1], [7, 2], [7, 7], [7, 6]]
position = [[7, 4]]

# Borders of scan angle
scan_angle = [[65, 300]]


# create the Robot instance.
robot = Robot()

print("Hello World!")


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


print("Hello World!")


can_bot = Canbot(robot,'motor_left', 'motor_right', 'dst_front_can', 'dst_front_bot', 'compass', 'infra_left', 'infra_right', 64 ,  3, 0.5 , 2, storage_positions, position, 'default_alignment',scan_angle)



while can_bot.time_step != -1:
	print("Hello World!")
	# Initiate scan
	can_bot.travel(1)

	# Exit if there are no more cans
	# if len(cans) == 0:
	# 	#wb_console_print(sprintf('No more cans to pick up'), WB_STDOUT);
	# 	can_bot.align(can_bot.default_alignment)
	# 	break
	
	# # Pickup cans 
	# for can in cans:
	# 	can_bot.go_coordinates(can)
	
	# # Store cans
	# can_bot.store_cans()
