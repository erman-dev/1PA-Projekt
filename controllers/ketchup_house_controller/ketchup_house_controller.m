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
wb_motor_set_position(motor_right, inf);
wb_motor_set_velocity(motor_left, 0);
wb_motor_set_velocity(motor_right, 0);


compass = wb_robot_get_device('compass');
wb_compass_enable(compass, TIME_STEP)
dst_front = wb_robot_get_device('dst_front');

distance_prev = 10000
nearest_cans=[];
nearest = [];
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

 angle = wb_compass_get_values(compass);
 wb_console_print(sprintf('Compass value is %f\n', angle(3)), WB_STDOUT); 
 
    
if angle(3) < 2
 while wb_robot_step(TIME_STEP) ~= -1
    wb_distance_sensor_enable(dst_front, TIME_STEP)
    wb_motor_set_velocity(motor_left, -1);
    wb_motor_set_velocity(motor_right, 1);
    
    distance = wb_distance_sensor_get_value(dst_front);
   
    angle = wb_compass_get_values(compass);
    %wb_console_print(sprintf('Compass value is %f\n', angle(3)), WB_STDOUT);
    
    
    if abs(diff([distance distance_prev])) > 0 && distance ~= 1000
    
     wb_console_print(sprintf('diff is %f\n', diff([distance distance_prev])), WB_STDOUT);
    
    nearest_cans = [nearest_cans; distance angle(3)];
    end
    distance_prev = distance
    
    
   % wb_console_print(sprintf('nearest_cans is %f\n', nearest_cans), WB_STDOUT);
    if angle(3) > 0.88
        wb_distance_sensor_disable(dst_front)
       wb_motor_set_velocity(motor_left, 0);
       wb_motor_set_velocity(motor_right, 0);
      
        can_distances = [nearest_cans(:,1)];
        sorted_can_distances = sort(can_distances);
        nearest = [];

         for i = 1:3
           can_position_in_matrix =  find(nearest_cans == sorted_can_distances(i));
           nearest = [nearest; nearest_cans(can_position_in_matrix, :)];
         end
          cans_to_deliver = nearest (1:3,:)

       wb_console_print(sprintf('deliver is %f\n', cans_to_deliver), WB_STDOUT);
     end
     
     
   
   
   
   
   end
end


  
  % send actuator commands, e.g.
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
