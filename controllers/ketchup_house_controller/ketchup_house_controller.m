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

can_bot = CanBot('motor_left', 'motor_right', 'dst_front', ...
                 'compass', 'infra_left', 'infra_right', ...
                 robot_position, storage_positions, 32);

%logging("INFO", 'I am travelling %d lines forward', 64);

can_bot.go_coordinates([1 1]);
can_bot.store_cans();
can_bot.go_coordinates([1 7]);
can_bot.store_cans();