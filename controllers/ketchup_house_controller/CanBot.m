classdef CanBot < handle
    properties (Access = private)
        motor_left
        motor_right
        dst_front
        compass
        infra_left 
        infra_right 
        TIME_STEP = 64
        bearing
        storage_positions
        turn_slow_down_angle = 15  %deg
        turn_angle_precision = 5 %deg
        speed_default = 6   	   %rad/s
        diff_threshold = 50        %ir sensor value
        position = [7 4]           %7th row 4th column
    end

    methods
        function h = CanBot(motor_left_handle, motor_right_handle, dst_front_handle, ...
                            compass_handle, infra_left_handle, infra_right_handle, storage_positions)
            
            % Set positions in which the robot will store cans
            h.storage_positions = storage_positions;

            % Get all robot devices 
            h.motor_left = wb_robot_get_device(motor_left_handle);
            h.motor_right = wb_robot_get_device(motor_right_handle);
            h.compass = wb_robot_get_device(compass_handle);
            h.dst_front = wb_robot_get_device(dst_front_handle);
            h.infra_left =  wb_robot_get_device(infra_left_handle);
            h.infra_right =  wb_robot_get_device(infra_right_handle);

            % Enable all sensors
            wb_distance_sensor_enable(h.dst_front, h.TIME_STEP);
            wb_distance_sensor_enable(h.infra_left, h.TIME_STEP);
            wb_distance_sensor_enable(h.infra_right, h.TIME_STEP);
            wb_compass_enable(h.compass, h.TIME_STEP);

            % Set motor positions
            wb_motor_set_position(h.motor_left, inf );
            wb_motor_set_position(h.motor_right, inf);
            wb_motor_set_velocity(h.motor_left, 0);
            wb_motor_set_velocity(h.motor_right, 0);
        end

        function travel(h, steps)
            % Robot travel specified amount of lines (steps) forward
            % The default state is that the robot is standing on a line
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

            while wb_robot_step(h.TIME_STEP) ~= -1
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

                % Ignore if the robot detects a line in the first 10 measurements
                if (on_line && off_line && nr_measurements < 10)
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

            % Calculate the angular difference between two angle
            % in CW direction and in CCW direction
            ccw_angle = round(r_angle, -1) - t_angle;
            cw_angle = t_angle - round(r_angle, -1);

            if cw_angle < 0 
                cw_angle = cw_angle + 360;
            end

            if ccw_angle < 0 
                ccw_angle = ccw_angle + 360;
            end

            if (ccw_angle < cw_angle)
                rotation_direction = 1;
            else
                rotation_direction = -1;
            end

            wb_console_print(sprintf('CW %f, CCW %f', cw_angle, ccw_angle), WB_STDOUT);
            wb_console_print(sprintf('Robot angle %d', r_angle), WB_STDOUT);
            wb_console_print(sprintf('Target angle %d', t_angle), WB_STDOUT);

            wb_motor_set_velocity(h.motor_left, h.speed_default*rotation_direction*-1);
            wb_motor_set_velocity(h.motor_right, h.speed_default*rotation_direction);

            angle_prev = false;

            while wb_robot_step(h.TIME_STEP) ~= -1
                r_angle = h.get_angle();

                angle_diff = r_angle - t_angle;

                if (~angle_prev) 
                    angle_prev = angle_diff;
                end

                if (abs(r_angle - t_angle) < h.turn_slow_down_angle)
                    wb_motor_set_velocity(h.motor_left, h.speed_default*rotation_direction*-1/8)
                    wb_motor_set_velocity(h.motor_right, h.speed_default*rotation_direction/8)
                end

                if (abs(r_angle - t_angle) < h.turn_angle_precision)
                    wb_motor_set_velocity(h.motor_left, h.speed_default*rotation_direction*-1/20);
                    wb_motor_set_velocity(h.motor_right, h.speed_default*rotation_direction/20);
                end

                wb_console_print(sprintf('Anle %f', r_angle), WB_STDOUT);
                wb_console_print(sprintf('Diff %f Prev %f', angle_diff, angle_prev), WB_STDOUT);
                wb_console_print(sprintf('Diff num %f', abs(diff([angle_diff angle_prev]))), WB_STDOUT);

                if ( sign(angle_diff) ~= sign(angle_prev) || abs(diff([angle_diff angle_prev])) > 300 )
                    wb_motor_set_velocity(h.motor_left, 0);
                    wb_motor_set_velocity(h.motor_right, 0);
                    break;
                end

                angle_prev = angle_diff;
            end
        end

        function store_cans(h)
            storage = h.storage_positions(1,:);
            h.storage_positions(1,:) = [];

            storage_coords = storage(1:2);
            storage_alignment = storage(3:4);

            wb_console_print(sprintf('Saving cans in %d,%d alignment %d,%d', storage_coords, storage_alignment), WB_STDOUT);

            h.go_coordinates(storage_coords)
            h.align(storage_alignment)
            h.travel(-1);
            h.align([-1 0]);

        end

        function deg_bearing = get_angle(h)
            while wb_robot_step(h.TIME_STEP) ~= -1
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
            % Returns bearing of the robot in degrees
            % 0 degrees is when facing the enemy robot
            % and tha value increse clockwise
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
            % Calculates bearing between two points
            x_diff = target(1) - h.position(1); % 1 - 7 = -6
            y_diff = target(2) - h.position(2); % 1 - 4 = -3

            bearing = sign([x_diff y_diff]); % [-1 -1]
        end

        function go_coordinates(h, target_coords)

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

        function scan_cans(h)
            distance_prev = 10000;
            nearest_cans=[];
            nearest = [];
            localized_cans = zeros(7);
            while wb_robot_step(h.TIME_STEP) ~= -1
                angle = wb_compass_get_values(h.compass);
                wb_console_print(sprintf('Compass value is %f\n', angle(3)), WB_STDOUT);
                if angle(3) < 2
                    while wb_robot_step(h.TIME_STEP) ~= -1
                    wb_distance_sensor_enable(h.dst_front, h.TIME_STEP)
                    wb_motor_set_velocity(h.motor_left, -1);
                    wb_motor_set_velocity(h.motor_right, 1);
                    
                    distance = wb_distance_sensor_get_value(h.dst_front);
                    
                    angle = wb_compass_get_values(h.compass);
                    %wb_console_print(sprintf('Compass value is %f\n', angle(3)), WB_STDOUT);
                    
                    
                    if abs(diff([distance distance_prev])) > 0 && distance ~= 1000
                    
                        %wb_console_print(sprintf('diff is %f\n', diff([distance distance_prev])), WB_STDOUT);
                        %wb_console_print(sprintf('Compass value is %f\n', angle(3)), WB_STDOUT); 
                    nearest_cans = [nearest_cans; distance angle(3)];
                    end
                    distance_prev = distance;
                    
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
                            
                
                        for i = 1:3
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
                                if T(i, 2) <= (0.6)
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
                    end
                    end
                end 
            end
        end
    end
end 