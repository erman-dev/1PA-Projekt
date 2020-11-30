% MATLAB controller for Webots
% File:          ketchup_house_controller.m
% Date:
% Description:
% Author: Roman Krček, Karel Hejl
% Modifications:


TIME_STEP = 64;

% get and enable devices, e.g.:
%  camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
motor_left = wb_robot_get_device('motor_left');
motor_right = wb_robot_get_device('motor_right');
wb_motor_set_position(motor_left, -inf );
wb_motor_set_velocity(motor_left, -1);
wb_motor_set_position(motor_right, inf);
wb_motor_set_velocity(motor_right, 1);
dst_front = wb_robot_get_device('dst_front');
wb_distance_sensor_enable(dst_front, TIME_STEP)


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

while wb_robot_step(TIME_STEP) ~= -1

  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
