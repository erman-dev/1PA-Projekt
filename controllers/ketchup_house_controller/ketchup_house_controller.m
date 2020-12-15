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

scan_angle = [310 50];

can_bot = CanBot('motor_left', 'motor_right', 'dst_front', ...
                 'compass', 'infra_left', 'infra_right', ...
                 robot_position, storage_positions, scan_angle, ...
                 32);

% Example for moving nodes
while wb_robot_step(32) ~= -1
	node_can1 = wb_supervisor_node_get_from_def('can1');
	can1_trans_field = wb_supervisor_node_get_field(node, 'translation');
	wb_supervisor_field_set_sf_vec3f(can1_trans_field, [-0.45 0.038 -0.24]);
	wb_supervisor_node_reset_physics(node);
	if wb_robot_step(32) ~= -1
		break;
	end
end

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

	can_bot.store_cans();
end