classdef CanBot < handle
    properties (Access = private)
        motor_left                 % text handle of device
        motor_right                % text handle of device
        dst_front                  % text handle of device
        compass                    % text handle of device
        infra_left                 % text handle of device
        infra_right                % text handle of device
        time_step                  % time step of the simulation
        bearing                    % direction of the robot
        storage_positions          % positions in which to store cans
        turn_slow_down_angle       % angle in which robot slows down
        turn_angle_precision = 0.5 % precision of robot in turns deg
        speed_default = 4     	   % speed of robot in rad/s
        diff_threshold = 50        % line detection threshold value
        position                   % current position of robot
    end

    methods
        function h = CanBot(motor_left_handle, motor_right_handle, ...
                            dst_front_handle,  compass_handle, ...
                            infra_left_handle, infra_right_handle, ...
                            position, storage_positions, time_step)
            %% Initiator takes in all device text handles and some
            %  additional configuration.
            %% Gets all devices and stores those hadles in the class,
            %  enables sensors and compass
            arguments
                motor_left_handle char
                motor_right_handle char
                dst_front_handle char
                compass_handle char
                infra_left_handle char
                infra_right_handle char
                position (1,2) double
                storage_positions (:,4) double
                time_step double = 64
            end
            
            % Set positions in which the robot will store cans
            h.storage_positions = storage_positions;
            h.time_step = time_step;
            h.position = position;

            % Get all robot devices 
            h.motor_left = wb_robot_get_device(motor_left_handle);
            h.motor_right = wb_robot_get_device(motor_right_handle);
            h.compass = wb_robot_get_device(compass_handle);
            h.dst_front = wb_robot_get_device(dst_front_handle);
            h.infra_left =  wb_robot_get_device(infra_left_handle);
            h.infra_right =  wb_robot_get_device(infra_right_handle);

            % Enable all sensors
            %wb_distance_sensor_enable(h.dst_front, h.time_step);
            wb_distance_sensor_enable(h.infra_left, h.time_step);
            wb_distance_sensor_enable(h.infra_right, h.time_step);
            wb_compass_enable(h.compass, h.time_step);

            % Set motor positions
            wb_motor_set_position(h.motor_left, inf );
            wb_motor_set_position(h.motor_right, inf);
            wb_motor_set_velocity(h.motor_left, 0);
            wb_motor_set_velocity(h.motor_right, 0);

            % Calculate variables
            h.turn_slow_down_angle = h.speed_default * 1.5;
        end

        function travel(h, steps)
            %% Robot travels specified amount of lines (steps) forwards
            %  or backwards
            arguments
                h
                steps {mustBeInteger}
            end

            steps_to_travel = steps;
            direction = sign(steps);
            robot_bearing = h.get_bearing();
            ir_prev = 1000;
            on_line = false;
            off_line = false;

            wb_console_print(sprintf('I am travelling %d lines forward', steps), WB_STDOUT);

            wb_motor_set_velocity(h.motor_left, h.speed_default*direction);
            wb_motor_set_velocity(h.motor_right, h.speed_default*direction);

            nr_measurements = 0;

            while wb_robot_step(h.time_step) ~= -1
                ir_left = wb_distance_sensor_get_value(h.infra_left);
                ir_right = wb_distance_sensor_get_value(h.infra_right);
                ir_rep = mean([ir_left ir_right]);
                ir_diff = diff([ir_prev ir_rep]);
                nr_measurements = nr_measurements + 1;

                %wb_console_print(sprintf('DEBUG: ir_rep %f', ir_rep), WB_STDOUT);
                %wb_console_print(sprintf('DEBUG: ir_diff %f', ir_diff), WB_STDOUT);
                    
                if (ir_diff > h.diff_threshold)
                    on_line = true;
                elseif (ir_diff < h.diff_threshold && on_line) 
                    off_line = true;
                end

                % Ignore if the robot detects a line in the first few measurements
                if (on_line && off_line && nr_measurements < h.speed_default*2)
                    on_line = false;
                    off_line = false;

                elseif (on_line && off_line)
                    h.position = h.position + robot_bearing*direction;
                    steps_to_travel = steps_to_travel - sign(steps_to_travel);
                    wb_console_print(sprintf('I have reached a line. %d to go', steps_to_travel), WB_STDOUT);
                    on_line = false;
                    off_line = false;
                end

                if (steps_to_travel == 0)
                    wb_motor_set_velocity(h.motor_left, 0);
                    wb_motor_set_velocity(h.motor_right, 0);
                    break;
                end

                ir_prev = ir_rep;
            end

            wb_console_print(sprintf('My position is now [%f,%f]', h.position(1), h.position(2)), WB_STDOUT);
        end

        function align(h, bearing)
            %% Aligns the robot alow with one axis
            arguments
                h
                bearing (1,2) {mustBeInteger}
            end

            if ( isequal(bearing, [-1 0]))
                t_angle = 0;
            elseif ( isequal(bearing, [0 1]))
                t_angle = 90;
            elseif ( isequal(bearing, [1 0]))
                t_angle = 180;
            elseif ( isequal(bearing, [0 -1]))
                t_angle = 270;
            else
                error('Invalid bearing value [%d,%d]', bearing(1), bearing(2));
            end

            r_angle = h.get_angle();    

            if ( abs(r_angle - t_angle) < 5 )
                wb_console_print(sprintf('No aligmnet needed'), WB_STDOUT);
                return;
            end

            [cwa, ccwa] = h.get_angle_diff(r_angle, t_angle);

            if ( round(ccwa, -1) < round(cwa, -1) )
                rotation_direction = 1;
            else
                rotation_direction = -1;
            end

            %wb_console_print(sprintf('CW %f, CCW %f', cwa, ccwa), WB_STDOUT);
            %wb_console_print(sprintf('Robot angle %d', r_angle), WB_STDOUT);
            %wb_console_print(sprintf('Target angle %d', t_angle), WB_STDOUT);

            wb_motor_set_velocity(h.motor_left, h.speed_default*rotation_direction*-1);
            wb_motor_set_velocity(h.motor_right, h.speed_default*rotation_direction);

            while wb_robot_step(h.time_step) ~= -1
                r_angle = h.get_angle();

                [cwa, ccwa] = h.get_angle_diff(r_angle, t_angle);

                angle_remaining = min([cwa ccwa]);

                if ( ccwa < cwa )
                    rotation_direction = 1;
                else
                    rotation_direction = -1;
                end

                if ( abs(angle_remaining) < h.turn_slow_down_angle )
                    slow_coef = max([10 20/angle_remaining]);
                    wb_motor_set_velocity(h.motor_left, h.speed_default*rotation_direction*-1/slow_coef);
                    wb_motor_set_velocity(h.motor_right, h.speed_default*rotation_direction/slow_coef);
                end
                
                if ( abs(angle_remaining) < h.turn_angle_precision )
                    wb_motor_set_velocity(h.motor_left, 0);
                    wb_motor_set_velocity(h.motor_right, 0);
                    break;
                end
                
                % wb_console_print(sprintf('CW %f, CCW %f', cwa, ccwa), WB_STDOUT);
                % wb_console_print(sprintf('Remainning %f Prev %f', angle_remaining, angle_prev), WB_STDOUT);
            end
        end

        function store_cans(h)
            %% Navigates the robot to a free storage position and
            %  backs one line to dump the cans
            storage = h.storage_positions(1,:);
            h.storage_positions(1,:) = [];

            storage_coords = storage(1:2);
            storage_alignment = storage(3:4);

            wb_console_print(sprintf('Saving cans in %d,%d alignment %d,%d', storage_coords, storage_alignment), WB_STDOUT);

            h.go_coordinates(storage_coords)
            h.align(storage_alignment)
            h.travel(abs(4 - storage_coords(2))*-1);
            h.align([-1 0]);

        end

        function [cw_angle, ccw_angle] = get_angle_diff(~, r_angle, t_angle)
            %% Calculates the angular difference between two angles
            %  in CW direction and in CCW direction
            arguments
                ~
                r_angle {mustBeReal}
                t_angle {mustBeReal}
            end

            ccw_angle = r_angle - t_angle;
            cw_angle = t_angle - r_angle;
            
            if cw_angle < 0 
                cw_angle = cw_angle + 360;
            end

            if ccw_angle < 0 
                ccw_angle = ccw_angle + 360;
            end

        end

        function deg_bearing = get_angle(h)
            %% Gets the angle of the robot relative to the world
            %  0   -> -Z direction in WB world
            %  90  -> +X direction in WB world
            %  180 -> +Z direction in WB world
            %  270 -> -X direction in WB world
            while wb_robot_step(h.time_step) ~= -1
                comapss_vals = wb_compass_get_values(h.compass);
                if (~isnan(comapss_vals))
                    break;
                end
            end

            rad = atan2(comapss_vals(1), comapss_vals(3));
            deg_bearing = (rad - 1.5708) / pi * 180.0;
            if (deg_bearing < 0.0)
                deg_bearing = deg_bearing + 360.0;                
            end
        end

        function bearing = get_bearing(h)
            %% Gets the axis and direction the robot is facing

            deg_bearing = h.get_angle();

            % wb_console_print(sprintf('Deg bearing %d', deg_bearing), WB_STDOUT);

            if ( abs(deg_bearing - 0) < 5 || abs(deg_bearing - 360) < 5 )
                bearing = [-1 0];
            elseif (abs(deg_bearing - 90) < 5)
                bearing = [0 1];
            elseif (abs(deg_bearing - 180) < 5)
                bearing = [1 0];
            elseif (abs(deg_bearing - 270) < 5)
                bearing = [0 -1];
            else
                error("Robot is misaligned!");
            end
        end

        function bearing = target_bearing(h, target)
            %% Calculates bearing between two points
            x_diff = target(1) - h.position(1); % 1 - 7 = -6
            y_diff = target(2) - h.position(2); % 1 - 4 = -3

            bearing = sign([x_diff y_diff]); % [-1 -1]
        end

        function go_coordinates(h, target_coords)
            %% Navigates the robot to specified coordinates
            arguments
                h
                target_coords (1,2) {mustBeInteger, mustBePositive}
            end

            target_bearing = h.target_bearing(target_coords);
            robot_bearing = h.get_bearing();

            x_diff = abs(target_coords(1) - h.position(1));
            y_diff = abs(target_coords(2) - h.position(2)); 

            % When the matching_coordinate is
            % -> 1: the the robot needs to travel along x axis first
            % -> 2: the the robot needs to travel along y axis first
            % -> 0: the robot is completly missaligned
            matching_coordinate = find(robot_bearing == target_bearing);

            wb_console_print(sprintf('Traveling from %d,%d to %d,%d target bearing %d,%d', h.position(1), h.position(2), target_coords(1), target_coords(2), target_bearing(1), target_bearing(2)), WB_STDOUT);

            % The robot is not aligned in any axis
            if (isempty(matching_coordinate))
                [~, t_axis, t_bearing] = find(target_bearing);

                wb_console_print(sprintf('DEBUG: t_axis [%d %d] ', t_axis), WB_STDOUT);
                wb_console_print(sprintf('DEBUG: t_bearing [%d %d] ', t_bearing), WB_STDOUT)

                if (t_axis == 1)
                    h.align([t_bearing 0]);
                    h.travel(x_diff);

                % travel only by y axis
                elseif (t_axis == 2)
                    h.align([0 t_bearing]);
                    h.travel(y_diff);
                
                % robot is totally misaligned, align by y
                elseif (isequal(t_axis, [1 2]))
                    h.align([0 t_bearing(2)]);
                    h.travel(y_diff);
                    h.align(h.target_bearing(target_coords));
                    h.travel(x_diff);
                end
                
            elseif (matching_coordinate == 1)
                wb_console_print(sprintf('X travel first'), WB_STDOUT);
                h.travel(x_diff);
                h.align(h.target_bearing(target_coords));
                h.travel(y_diff);

            elseif (matching_coordinate == 2)     
                wb_console_print(sprintf('Y travel first'), WB_STDOUT);
                h.travel(y_diff);
                h.align(h.target_bearing(target_coords));
                h.travel(x_diff)

            end

            wb_console_print(sprintf('End move'), WB_STDOUT);
        end

        function localized_cans_coords = scan_cans(h)
            % Gets coordinates of nearest cans
            distance_prev = 10000;
            nearest_cans=[];
            nearest = [];
            localized_cans = zeros(7);
            localized_cans_coords = [];

            % Turn robot to 50°
            wb_motor_set_velocity(h.motor_left, h.speed_default);
            wb_motor_set_velocity(h.motor_right, -h.speed_default);
            while wb_robot_step(h.time_step) ~= -1
                r_angle = h.get_angle();
                if r_angle > 50
                    wb_motor_set_velocity(h.motor_left, 0);
                    wb_motor_set_velocity(h.motor_right, 0);
                    break;
                end
            end

            while wb_robot_step(h.time_step) ~= -1
                angle = wb_compass_get_values(h.compass);
                wb_console_print(sprintf('Compass value is %f\n', angle(3)), WB_STDOUT);
                if angle(3) < 2
                    while wb_robot_step(h.time_step) ~= -1
                    wb_distance_sensor_enable(h.dst_front, h.time_step)
                    wb_motor_set_velocity(h.motor_left, -1);
                    wb_motor_set_velocity(h.motor_right, 1);
                    
                    distance = wb_distance_sensor_get_value(h.dst_front);
                    
                    angle = wb_compass_get_values(h.compass);
                    %wb_console_print(sprintf('Compass value is %f\n', angle(3)), WB_STDOUT);
                    
                    
                    if abs(diff([distance distance_prev])) > 50 && distance ~= 1000
                        %wb_console_print(sprintf('diff is %f\n', diff([distance distance_prev])), WB_STDOUT);
                        wb_console_print(sprintf('Can detected %f angle %f', distance, angle(3)), WB_STDOUT); 
                        nearest_cans = [nearest_cans; distance angle(3)];
                    end
                    
                    distance_prev = distance;
                    
                    % wb_console_print(sprintf('nearest_cans is %f\n', nearest_cans), WB_STDOUT);
                    if angle(3) > 0.88
                        wb_distance_sensor_disable(h.dst_front)
                        wb_motor_set_velocity(h.motor_left, 0);
                        wb_motor_set_velocity(h.motor_right, 0);

                        % No cans detected
                        if ( isempty(nearest_cans) )
                            return;
                        end

                        nearest_cans
                        
                        can_distances = [nearest_cans(:,1)];
                        sorted_can_distances = sort(can_distances)
                        nearest = [];
                
                        if size(sorted_can_distances, 1) < 3
                            n = size(sorted_can_distances, 1);
                        else
                            n = 3;
                        end   

                        wb_console_print(sprintf('Choosing %d. \n', n), WB_STDOUT); 

                        for i = 1:n
                            can_position_in_matrix =  find(nearest_cans == sorted_can_distances(i));
                            nearest = [nearest; nearest_cans(can_position_in_matrix, :)];
                        end
                        
                        cans_to_deliver = nearest(1:n,:)
                
                        for i = 1:n
                            if cans_to_deliver(i, 2) < (0.1) && cans_to_deliver(i, 2) > (-0.2)
                                if cans_to_deliver(i, 1) == 300
                                localized_cans(5, 4) = 1;
                                elseif cans_to_deliver(i, 1) == 600
                                localized_cans(4, 4) = 1; 
                                elseif cans_to_deliver(i, 1) == 800
                                localized_cans(3, 4) = 1;
                                end
                            elseif cans_to_deliver(i, 2) >= (0.1)
                                if cans_to_deliver(i, 1) == 400
                                    localized_cans(5, 3) = 1;
                                elseif cans_to_deliver(i, 1) == 500
                                    localized_cans(5, 2) = 1; 
                                elseif cans_to_deliver(i, 1) == 600
                                    localized_cans(4, 3) = 1;
                                elseif cans_to_deliver(i, 1) == 900
                                    localized_cans(3, 2) = 1;
                                elseif cans_to_deliver(i, 1) == 700 
                                if cans_to_deliver(i, 2) <= (0.6)
                                    localized_cans(4, 2) = 1;
                                elseif cans_to_deliver(i, 2) > (0.6)
                                    localized_cans(5, 1) = 1;  
                                end
                            elseif cans_to_deliver(i, 1) == 800
                                if cans_to_deliver(i, 2) <= (0.4)
                                localized_cans(3, 3) = 1;  
                            elseif cans_to_deliver(i, 2) > (0.4)
                                localized_cans(4, 1) = 1;  
                                end
                            end
                        elseif cans_to_deliver(i, 2) <= (-0.2)
                                if cans_to_deliver(i, 1) == 400
                                localized_cans(5, 5) = 1;
                                elseif cans_to_deliver(i, 1) == 500
                                localized_cans(5, 6) = 1; 
                                elseif cans_to_deliver(i, 1) == 600
                                localized_cans(4, 5) = 1;
                                elseif cans_to_deliver(i, 1) == 900
                                localized_cans(3, 6) = 1;
                                elseif cans_to_deliver(i, 1) == 700
                                if cans_to_deliver(i, 2) >= (-0.7)
                                    localized_cans(4, 6) = 1;
                                elseif cans_to_deliver(i, 2) < (-0.7)
                                    localized_cans(5, 7) = 1;
                                end
                                elseif cans_to_deliver(i, 1) == 800
                                if cans_to_deliver(i, 2) >= (-0.5)
                                    localized_cans(3, 5) = 1;  
                                elseif cans_to_deliver(i, 2) < (-0.5)
                                    localized_cans(4, 7) = 1;
                                end
                                end 
                            end
                        end

                        [a,b] = find(localized_cans==1);
                        localized_cans_coords = [a, b];

                        wb_console_print(sprintf('Before align'), WB_STDOUT);
                        h.align([-1 0]);
                        wb_console_print(sprintf('After align'), WB_STDOUT);

                        return
                    end
                    end
                end 
            end
        end
    end
end 