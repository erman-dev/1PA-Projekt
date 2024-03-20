"""controller_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from can_bot import CanBot

storage_positions = [[7, 1], [7, 2], [7, 7], [7, 6]]
robot_position = [[7, 4]]

# Borders of scan angle
scan_angle = [[65, 300]]

# create the Robot instance.
robot = Robot()
can_bot = CanBot(robot, sdfas)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1:
	# Initiate scan
	cans = can_bot.scan_cans()

	# Exit if there are no more cans
	if len(cans) == 0:
		#wb_console_print(sprintf('No more cans to pick up'), WB_STDOUT);
		can_bot.align(can_bot.default_alignment)
		break
	
	# Pickup cans 
	for can in cans:
		can_bot.go_coordinates(can)
	
	# Store cans
	can_bot.store_cans()
