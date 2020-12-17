classdef CanBot < handle

    properties
        motor_left                  % text handle of device
        motor_right                 % text handle of device
        dst_front_can               % text handle of device
        dst_front_bot               % text handle of device
        dst_left_bot                % text handle of device
        dst_right_bot               % text handle of device
        compass                     % text handle of device
        infra_left                  % text handle of device
        infra_right                 % text handle of device
        time_step                   % time step of the simulation
        bearing                     % direction of the robot
        storage_positions           % positions in which to store cans
        turn_slow_down_angle        % angle in which robot slows down
        turn_angle_precision = 0.5  % precision of robot in turns deg
        speed_default = 3           % speed of robot in rad/s
        position                    % current position of robot
        default_alignment           % default direction of the robot
        scan_angle                  % two angles between which to scan for cans
    end

    methods

        function h = CanBot(motor_left_handle, motor_right_handle, ...
                dst_front_can_handle, dst_front_bot_handle, ...
                dst_left_bot_handle, dst_right_bot_handle, ...
                compass_handle, ...
                infra_left_handle, infra_right_handle, ...
                position, storage_positions, scan_angle, ...
                time_step)
            %% Initiator takes in all device text handles and some
            %  additional configuration.
            %% Gets all devices and stores those hadles in the class,
            %  enables sensors and compass
            arguments
                motor_left_handle char
                motor_right_handle char
                dst_front_can_handle char
                dst_front_bot_handle char
                dst_left_bot_handle char
                dst_right_bot_handle char
                compass_handle char
                infra_left_handle char
                infra_right_handle char
                position (1, 2) double
                storage_positions (:, 2) double
                scan_angle (1, 2) double
                time_step double = 64
            end

            % Set positions in which the robot will store cans
            h.position = position;
            h.storage_positions = storage_positions;
            h.scan_angle = scan_angle;
            h.time_step = time_step;

            % Get all robot devices
            h.motor_left = wb_robot_get_device(motor_left_handle);
            h.motor_right = wb_robot_get_device(motor_right_handle);
            h.compass = wb_robot_get_device(compass_handle);
            h.dst_front_can = wb_robot_get_device(dst_front_can_handle);
            h.dst_front_bot = wb_robot_get_device(dst_front_bot_handle);
            h.dst_left_bot = wb_robot_get_device(dst_left_bot_handle);
            h.dst_right_bot = wb_robot_get_device(dst_right_bot_handle);
            h.infra_left = wb_robot_get_device(infra_left_handle);
            h.infra_right = wb_robot_get_device(infra_right_handle);

            % Enable all sensors
            wb_distance_sensor_enable(h.infra_left, h.time_step);
            wb_distance_sensor_enable(h.infra_right, h.time_step);
            wb_distance_sensor_enable(h.dst_front_bot, h.time_step);
            wb_distance_sensor_enable(h.dst_left_bot, h.time_step);
            wb_distance_sensor_enable(h.dst_right_bot, h.time_step);
            wb_compass_enable(h.compass, h.time_step);

            % Set motor positions
            wb_motor_set_position(h.motor_left, inf);
            wb_motor_set_position(h.motor_right, inf);
            wb_motor_set_velocity(h.motor_left, 0);
            wb_motor_set_velocity(h.motor_right, 0);

            % Calculate variables
            h.turn_slow_down_angle = h.speed_default * 1.5;
            h.default_alignment = h.get_bearing();
            h.default_alignment
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
            
            % Variables for collision avoidance
            nr_measurements = 0;
            d_front_bot_prev = 1000;
            d_left_bot_prev = 1000;
            d_right_bot_prev = 1000;
            robot_passing = false;
            roobt_incoming = true;
            d_diff_threshold = 600; % Distance sifference threshold
            d_dist_threshold = 300; % Threshold distance

            % Go!
            wb_motor_set_velocity(h.motor_left, h.speed_default * direction);
            wb_motor_set_velocity(h.motor_right, h.speed_default * direction);

            while wb_robot_step(h.time_step) ~= -1
                ir_left = wb_distance_sensor_get_value(h.infra_left);
                ir_right = wb_distance_sensor_get_value(h.infra_right);
                ir_rep = mean([ir_left ir_right]);
                ir_trend = sign(round(ir_rep, -1) - round(ir_prev, -1));
                nr_measurements = nr_measurements + 1;

                %wb_console_print(sprintf('DEBUG: ir_rep %f', ir_rep), WB_STDOUT);
                %wb_console_print(sprintf('DEBUG: ir_trend %f', ir_diff, ir_trend), WB_STDOUT);

                % Prevent colision with enemy robot
                d_front_bot = wb_distance_sensor_get_value(h.dst_front_bot);
                d_left_bot = wb_distance_sensor_get_value(h.dst_left_bot);
                d_right_bot = wb_distance_sensor_get_value(h.dst_right_bot);

                d_front_diff = diff([d_front_bot, d_front_bot_prev]);
                %d_left_diff = diff([d_left_bot, d_left_bot_prev]);
                %d_right_diff = diff([d_right_bot, d_right_bot_prev]);

                % Robot detected passing in front of us
                if (d_front_diff > d_diff_threshold)
                    robot_passing = true;
                elseif (d_front_bot < d_dist_threshold && ~robot_passing)
                    %robot_incoming = true;
                end

                % If the enemy robot is passing in front of us and we come close - stop
                if (robot_passing && d_front_bot < d_dist_threshold * 1.5)
                    wb_motor_set_velocity(h.motor_left, 0);
                    wb_motor_set_velocity(h.motor_right, 0);

                    while (wb_robot_step(h.time_step) ~= -1 && d_front_bot ~= 1000)
                        d_front_bot = wb_distance_sensor_get_value(h.dst_front_bot);
                    end

                    wb_motor_set_velocity(h.motor_left, h.speed_default * direction);
                    wb_motor_set_velocity(h.motor_right, h.speed_default * direction);
                    robot_passing = false;
                end

                % Start of line detected
                if (ir_trend == 1)
                    on_line = true;
                    % end of line detected
                elseif ((ir_trend == 0 || ir_trend == -1) && on_line)
                    off_line = true;
                end

                % Ignore if the robot detects a line in the first few measurements
                % after start and after every line
                if (on_line && off_line && nr_measurements < 120 / h.speed_default)
                    on_line = false;
                    off_line = false;

                    % Line detected
                elseif (on_line && off_line)
                    h.position = h.position + robot_bearing * direction;
                    steps_to_travel = steps_to_travel - sign(steps_to_travel);
                    wb_console_print(sprintf('I have reached a line. %d to go', steps_to_travel), WB_STDOUT);
                    on_line = false;
                    off_line = false;
                    nr_measurements = 0;
                end

                % Destination reached
                if (steps_to_travel == 0)
                    wb_motor_set_velocity(h.motor_left, 0);
                    wb_motor_set_velocity(h.motor_right, 0);
                    break;
                end

                ir_prev = ir_rep;
                d_front_bot_prev = d_front_bot;
                d_left_bot_prev = d_left_bot;
                d_right_bot_prev = d_right_bot;
            end

            wb_console_print(sprintf('My position is now [%f,%f]', h.position(1), h.position(2)), WB_STDOUT);
        end

        function align(h, bearing)
            %% Aligns the robot alow with one axis
            arguments
                h
                bearing (1, 2) {mustBeInteger}
            end

            if (isequal(bearing, [-1 0]))
                t_angle = 0;
            elseif (isequal(bearing, [0 1]))
                t_angle = 90;
            elseif (isequal(bearing, [1 0]))
                t_angle = 180;
            elseif (isequal(bearing, [0 -1]))
                t_angle = 270;
            else
                error('Invalid bearing value [%d,%d]', bearing(1), bearing(2));
            end

            h.turn(t_angle);

        end

        function turn(h, t_angle)
            %% Turns robot to specified angle
            arguments
                h
                t_angle {mustBeNonnegative, mustBeInteger}
            end

            r_angle = h.get_angle();

            if (abs(r_angle - t_angle) < 5)
                wb_console_print(sprintf('No aligmnet needed'), WB_STDOUT);
                return;
            end

            [cwa, ccwa] = h.get_angle_diff(r_angle, t_angle);

            if (round(ccwa, -1) < round(cwa, -1))
                rotation_direction = 1;
            else
                rotation_direction = -1;
            end

            %wb_console_print(sprintf('CW %f, CCW %f', cwa, ccwa), WB_STDOUT);
            %wb_console_print(sprintf('Robot angle %d', r_angle), WB_STDOUT);
            %wb_console_print(sprintf('Target angle %d', t_angle), WB_STDOUT);

            wb_motor_set_velocity(h.motor_left, h.speed_default * rotation_direction * -1);
            wb_motor_set_velocity(h.motor_right, h.speed_default * rotation_direction);

            % Chooses better rotation direction
            while wb_robot_step(h.time_step) ~= -1
                r_angle = h.get_angle();

                [cwa, ccwa] = h.get_angle_diff(r_angle, t_angle);

                angle_remaining = min([cwa ccwa]);

                if (ccwa < cwa)
                    rotation_direction = 1;
                else
                    rotation_direction = -1;
                end

                if (abs(angle_remaining) < h.turn_slow_down_angle)
                    slow_coef = max([10 20 / angle_remaining]);
                    wb_motor_set_velocity(h.motor_left, h.speed_default * rotation_direction * -1 / slow_coef);
                    wb_motor_set_velocity(h.motor_right, h.speed_default * rotation_direction / slow_coef);
                end

                if (abs(angle_remaining) < h.turn_angle_precision)
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
            storage_coords = h.storage_positions(1, :);
            h.storage_positions(1, :) = [];

            if storage_coords(2) > 4
                storage_alignment = [0 1];
            else
                storage_alignment = [0 -1];
            end

            wb_console_print(sprintf('Saving cans in %d,%d alignment %d,%d', storage_coords, storage_alignment), WB_STDOUT);

            h.go_coordinates(storage_coords)
            h.align(storage_alignment)
            h.travel(abs(4 - storage_coords(2)) * -1);
            h.align(h.default_alignment);
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
                compass_vals = wb_compass_get_values(h.compass);

                if (~isnan(compass_vals))
                    break;
                end

            end

            rad = atan2(compass_vals(1), compass_vals(3));
            deg_bearing = (rad - 1.5708) / pi * 180.0;

            if (deg_bearing < 0.0)
                deg_bearing = deg_bearing + 360.0;
            end

        end

        function bearing = get_bearing(h)
            %% Gets the axis and direction the robot is facing

            deg_bearing = h.get_angle();

            % wb_console_print(sprintf('Deg bearing %d', deg_bearing), WB_STDOUT);

            if (abs(deg_bearing - 0) < 5 || abs(deg_bearing - 360) < 5)
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
                target_coords (1, 2) {mustBeInteger, mustBePositive}
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

                %wb_console_print(sprintf('DEBUG: t_axis [%d %d] ', t_axis), WB_STDOUT);
                %wb_console_print(sprintf('DEBUG: t_bearing [%d %d] ', t_bearing), WB_STDOUT)

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

        function cans_pos = scan_cans(h)
            %% Gets coordinates of nearest cans to pickup

            wb_distance_sensor_enable(h.dst_front_can, h.time_step);
            distance_prev = 10000;
            nearest_cans = [];
            cans_pos = [];
            wb_console_print(sprintf('Scan angle is from %f to %f\n', h.scan_angle(1), h.scan_angle(2)), WB_STDOUT)

            % Turn robot to scan angle
            h.turn(h.scan_angle(1))

            %wb_console_print(sprintf('DEBUG: Turned to angle scan angle 1 - %d', h.scan_angle(1)), WB_STDOUT);

            %wb_console_print(sprintf('Angle is %f\n', h.get_angle()), WB_STDOUT)

            [cwa, ccwa] = h.get_angle_diff(h.scan_angle(1), h.scan_angle(2));

            if (ccwa < cwa)
                rotation_direction = 1;
            else
                rotation_direction = -1;
            end

            wb_motor_set_velocity(h.motor_left, h.speed_default / 4 * rotation_direction * -1);
            wb_motor_set_velocity(h.motor_right, h.speed_default / 4 * rotation_direction);

            while wb_robot_step(h.time_step) ~= -1

                distance_can = wb_distance_sensor_get_value(h.dst_front_can);
                distance_bot = wb_distance_sensor_get_value(h.dst_front_bot);
                r_angle = h.get_angle();

                %Ignores enemy robot and measure can distances and angles
                if (abs(diff([distance_can distance_prev])) > 50 && distance_can ~= 1000)

                    wb_console_print(sprintf('Can detected! Distance: %f angle %f', distance_can, r_angle), WB_STDOUT);

                    if (abs(distance_bot - distance_can) < 100 && distance_bot ~= 1000)
                        wb_console_print(sprintf('Never mind, it was a robot, distance: %f', distance_bot), WB_STDOUT);
                    else
                        nearest_cans = cat(1, nearest_cans, [distance_can r_angle]);
                    end

                end

                distance_prev = distance_can;

                %Evaluate nearest cans
                if abs(h.scan_angle(2) - r_angle) < 5
                    wb_distance_sensor_disable(h.dst_front_can);
                    wb_motor_set_velocity(h.motor_left, 0);
                    wb_motor_set_velocity(h.motor_right, 0);

                    % No cans detected
                    if isempty(nearest_cans)
                        return;
                    end

                    if size(nearest_cans, 1) < 3
                        n = size(nearest_cans, 1);
                    else
                        n = 3;
                    end

                    %wb_console_print(sprintf('Choosing %d. \n', n), WB_STDOUT);

                    [~, idx] = sort(nearest_cans(:, 1));
                    cans_to_deliver = nearest_cans(idx, :);
                    cans_to_deliver = cans_to_deliver(1:n, :);

                    wb_console_print(sprintf('Choosing %d. \n', cans_to_deliver), WB_STDOUT);

                    %Transfom length and angle to coordinates
                    i = 1;
                    square = 200; % dimension of one sqaure in mm
                    cans_pos = []; % empty array for the final can coordinates

                    while i ~= n + 1
                        can = cans_to_deliver(i, :);
                        y_diff = sind(can(2)) * can(1) - 30;
                        y_sqaure_diff = round(y_diff / square);
                        x_diff = cosd(can(2)) * can(1);
                        x_sqaure_diff = round(x_diff / square);

                        can_rel_pos = [x_sqaure_diff * -1, y_sqaure_diff];
                        can_abs_pos = h.position + can_rel_pos;
                        cans_pos = cat(1, cans_pos, can_abs_pos);
                        i = i + 1;

                    end

                    %wb_console_print(sprintf('Before align'), WB_STDOUT);
                    h.align(h.default_alignment);
                    %wb_console_print(sprintf('After align'), WB_STDOUT);

                    return
                end

            end

        end

    end

end
