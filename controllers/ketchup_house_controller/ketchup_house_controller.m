% MATLAB controller for Webots
% File:          ketchup_house_controller.m
% Date:
% Description:
% Author: Roman Krček, Karel Hejl

%desktop;
%keyboard;

% Example position matrix
% 1 - represents cans
% 2 - represents robot
% 0 0 0 2 0 0 0
% 0 0 0 0 0 0 0
% 0 1 0 1 0 1 0
% 0 0 0 0 0 0 0
% 0 0 0 0 0 0 0
% 0 0 0 0 0 0 0
% 0 0 0 0 0 0 0

storage_positions = [7 1, 0 -1;
                     7 2, 0 -1;
                     7 7, 0  1;
                     7 6, 0  1];
robot_position = [7 4];

scan_angle = [65 300]

can_bot = CanBot('motor_left', 'motor_right', 'dst_front', ...
                 'compass', 'infra_left', 'infra_right', ...
                 robot_position, storage_positions, scan_angle, ...
                 32);


% Man can collection program
while true
	
	cans = can_bot.scan_cans()

	if isempty(cans)
		wb_console_print(sprintf('No more cans to pick up'), WB_STDOUT);
		can_bot.align([-1 0]);
		break;
	end

	for i = 1:size(cans, 1)
		target_coords = cans(i, :);
		can_bot.go_coordinates(target_coords);
	end
	
	can_bot.store_cans()
	
end