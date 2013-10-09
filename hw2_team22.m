function bug2(serPort)

    %=============================================================%
    % Description                                                 %
    %=============================================================%
    % This is a simple solution for Homework 1.                   %
    % The robot moves forward till it bumps on an object (wall).  %
    % The robot follows the object around till it reaches the     %
    % first bump position and then returns back to its initial    %
    % starting point with the same orientation.                   %
    %=============================================================%
    
    %%
    %=============================================================%
    % Clear cache & avoid NaN values                              %
    %=============================================================%
    clc;                                                          % Clear the cache
    
    % Poll for bump Sensors to avoid getting NaN values when the 
    % robot first hits a wall
    [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
              BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    %=============================================================%

    %%
    %=============================================================%
    % Variable Declaration                                        %
    %=============================================================%
    
    %%
    %=======================%
    % Position Declaration  %
    %=======================%

    % Current Position
    current_pos_x = 0;
    current_pos_y = 0;
    current_angle = 0;
    
    % First Hit Position
    first_hit_pos_x = 0;
    first_hit_pos_y = 0;
    first_hit_angle = 0;
    bump_object_y = 0;
    %%
    %=======================%
    % Velocity Declaration  %
    %=======================%
    velocity_val = 0.2;
    angular_velocity_val = 0.1;
    %=======================%

    %=======================%
    % Distance Thresholds   %
    %=======================%
    dist_from_start_point = 0.3;
    dist_from_first_hit_point = 0.2;
    %=======================%
   
    %=======================%
    % Status Variable       %
    %=======================%
    status = 1;             % 1 -> Move Forward, 
                            % 2 -> Wall Follow | Havent left the threshold of the hit point
                            % 3 -> Wall Follow | Left the threshold of the hit point
                            % 4 -> Go Back to Start Position  
                            % 5 -> Stop and Orient at Start Position
    %=============================================================%
    
    x_hist = [current_pos_x]
    y_hist = [current_pos_y]

    %% Main Loop
    while 1 
        
        %%
        %=============================================================%
        % Step 1 - Read Sensors                                       %
        %=============================================================%
        % For each loop, first read data from the robot sensor
        [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
              BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);     % Poll for Bump Sensors
        Wall = WallSensorReadRoomba(serPort);                         % Poll for Wall Sensor
        distance_temp = DistanceSensorRoomba(serPort);                % Poll for Distance delta
        angle_temp = AngleSensorRoomba(serPort);                      % Poll for Angle delta
        %=============================================================%
        
        %%
        %=============================================================%
        % Step 2 - Update Odometry                                    %
        %=============================================================%
        % Keep tracking the position and angle before the first hit
        current_angle = current_angle + angle_temp;               
        current_pos_x = current_pos_x + sin(current_angle) * distance_temp;
        current_pos_y = current_pos_y + cos(current_angle) * distance_temp;
        
        % Keep tracking the position and angle after the first hit
        first_hit_angle = first_hit_angle + angle_temp;
        first_hit_pos_x = first_hit_pos_x + sin(first_hit_angle) * distance_temp;
        first_hit_pos_y = first_hit_pos_y + cos(first_hit_angle) * distance_temp;

        x_hist = [x_hist current_pos_x];
        y_hist = [y_hist current_pos_y];

        fprintf('current_angle:%d, current_pos_x: %d, current_pos_y: %d\n', current_angle, current_pos_x, current_pos_y)
        fprintf('first_hit_angle:%d, first_hit_pos_x: %d, first_hit_pos_y: %d\n', first_hit_angle, first_hit_pos_x, first_hit_pos_y)
        drawnow;
        %=============================================================%
        
        %%
        %=============================================================%
        % Step 3 - Calculate Euclidean Distances                      %
        %=============================================================%
        start_distance = sqrt(current_pos_x ^ 2 + current_pos_y ^ 2);
        hit_distance = sqrt(first_hit_pos_x ^ 2 + first_hit_pos_y ^ 2);
        %=============================================================%
        
        %%
        %=============================================================%
        % Step 4 - Check Status                                       %
        %=============================================================%
        switch status
            case 1 % Move Forward
                display('Moving Forward');
                SetFwdVelAngVelCreate(serPort, velocity_val, 0 );
                if abs(current_pos_y) >= 10.0
                    status = 5;
                end
                if (BumpRight || BumpLeft || BumpFront)
                    status = 2; % Change Status to Wall Follow
                    first_hit_angle = 0;
                    first_hit_pos_x = 0;
                    first_hit_pos_y = 0;
                    bump_object_y = current_pos_y;                    
                end
            case 2 % Wall Follow | Haven't left the threshold of the hit point
                display('Block 2');
                WallFollow(velocity_val, angular_velocity_val, BumpRight, BumpLeft, BumpFront, Wall, serPort);
                if (hit_distance > dist_from_first_hit_point)
                    status = 3;
                end
            case 3 % Wall Follow | Left the threshold of the hit point
                display('Block 3');
                WallFollow(velocity_val, angular_velocity_val, BumpRight, BumpLeft, BumpFront, Wall, serPort);
                if((current_pos_x <= 0) && (current_pos_y > bump_object_y + 0.5))
                    fprintf('reached x')
                %if(hit_distance < dist_from_first_hit_point)
                   status = 4; 
                end
            case 4 % Go Back to Start Position
                display('Block 4');                
                turnAngle(serPort, angular_velocity_val, current_angle);
                current_angle = mod(current_angle, pi) + pi;
                if (abs(pi - current_angle) <= 0.1)
                    current_angle = 0;
                    SetFwdVelAngVelCreate(serPort, 0, 0 );
                    if abs(current_pos_y) < 10.0
                        status = 1;
                    else
                        status = 5;
                    end
                end
            case 5 % Stop and Orient at Start Position
                fprintf('reached finish\n')
                fprintf('plotting...\n')
                plot(x_hist, y_hist)
                title('x and y traveled')
                xlabel('x')
                ylabel('y')
                return;
        end
    end
end

%%
% Wall Follow Function
function WallFollow(velocity_val, angular_velocity_val, BumpRight, BumpLeft, BumpFront, Wall, serPort)

    % Angle Velocity for different bumps
    av_bumpright =  4 * angular_velocity_val;
    av_bumpleft  =  2 * angular_velocity_val;
    av_bumpfront =  3 * angular_velocity_val;
    av_nowall    = -4 * angular_velocity_val;
    
    if BumpRight || BumpLeft || BumpFront
        v = 0;                              % Set Velocity to 0
    elseif ~Wall
        v = 0.25 * velocity_val;            % Set Velocity to 1/4 of the default
    else
        v = velocity_val;                   % Set Velocity to the default
    end

    if BumpRight
    av = av_bumpright;                      % Set Angular Velocity to av_bumpright
    elseif BumpLeft
        av = av_bumpleft;                   % Set Angular Velocity to av_bumpleft
    elseif BumpFront
        av = av_bumpfront;                  % Set Angular Velocity to av_bumpfront
    elseif ~Wall
        av = av_nowall;                     % Set Angular Velocity to av_nowall
    else
        av = 0;                             % Set Angular Velocity to 0
    end
    SetFwdVelAngVelCreate(serPort, v, av );
end