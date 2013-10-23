% HW3 Team 22
% Sophie Chou sbc2125
% Arvind Srinivasan vs2371
%

function map_room(serPort, t_max)

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
    tic

    ROOMBA_UNIT = .35

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
    room = [1; 0]
    y_0 = 1;
    x_0 = 1;
    %% Main Loop    
    while(toc < t_max)
        
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
        current_pos_x = current_pos_x + cos(current_angle) * distance_temp;
        current_pos_y = current_pos_y + sin(current_angle) * distance_temp;
        
        % Keep tracking the position and angle after the first hit
        first_hit_angle = first_hit_angle + angle_temp;
        first_hit_pos_x = first_hit_pos_x + cos(first_hit_angle) * distance_temp;
        first_hit_pos_y = first_hit_pos_y + sin(first_hit_angle) * distance_temp;


        previous_x = floor(current_pos_x/ROOMBA_UNIT)
        previous_y = floor(current_pos_y/ROOMBA_UNIT)

        if(floor(current_pos_x/ROOMBA_UNIT) > previous_x)
            x_hist = [x_hist previous_x]         
        end
   
        if(floor(current_pos_y/ROOMBA_UNIT) > previous_y)
            y_hist = [y_hist previous_y]

        end

        dims  = size(room)
        % current size of matrix
        y_dim = dims(1)
        x_dim = dims(2)
        if (previous_y > y_dim) % add a row to the top
            room = [zeros(1, x_dim); room]
        elseif (previous_y + y_0 < 1) % add a row to the bottom
            room = [room; zeros(1, x_dim)]
            y_0 = y_0 + 1
        end
        if (previous_x > x_dim) % add a row to the right
            room = [room zeros(y_dim, 1)]
        elseif (previous_x + x_0  < 1) % add a row to the left
            room = [zeros(y_dim, 1) room]
            x_0 = x_0 + 1
        end

        disp('PREV X AND Y')
        disp(x_0)
        disp(y_0)
        disp(previous_x)
        disp(previous_y)

        room(previous_y + y_0, previous_x + x_0) = 1

        imagesc(room)
        grid on


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
                %display('Moving Forward');
                SetFwdVelAngVelCreate(serPort, velocity_val, 0 );

                if (BumpRight || BumpLeft || BumpFront)
                    status = 2; % Change Status to Wall Follow
                    first_hit_angle = 0;
                    first_hit_pos_x = 0;
                    first_hit_pos_y = 0;
                    bump_object_y = current_pos_y;                    
                end
            case 2 % Wall Follow | Havent left the threshold of the hit point
                %display('Block 2');
                WallFollow(velocity_val, angular_velocity_val, BumpRight, BumpLeft, BumpFront, Wall, serPort);
                if (hit_distance > dist_from_first_hit_point)
                    status = 3;
                end
            case 3 % Wall Follow | Left the threshold of the hit point
                %display('Block 3');
                WallFollow(velocity_val, angular_velocity_val, BumpRight, BumpLeft, BumpFront, Wall, serPort);
                if((current_pos_x <= 0) && (current_pos_y > bump_object_y + 0.5))
                    %fprintf('reached x')
                %if(hit_distance < dist_from_first_hit_point)
                   status = 4; 
                end
            
            case 4 %randomnly go somewhere else
                turnAngle(serPort, angular_velocity_val, rand * 2 * pi )
                status = 1;

            end

            %case 4 % Go Back to Start Position
                %display('Block 4');                
                %turnAngle(serPort, angular_velocity_val, current_angle);
                %current_angle = mod(current_angle, pi) + pi;
                %if (abs(pi - current_angle) <= 0.1)
                %    current_angle = 0;
                %    SetFwdVelAngVelCreate(serPort, 0, 0 );
                %    if abs(current_pos_y) < 10.0
                %        status = 1;
                %    else
                %        status = 5;
                %     end
                %end
            %case 5 % Stop and Orient at Start Position
            %    fprintf('reached finish\n')
            %    fprintf('plotting...\n')
            %    plot(x_hist, y_hist)
            %    title('x and y traveled')
            %    xlabel('x')
            %    ylabel('y')
        %end
    end
    fprintf('ran out of time\n')
    %fprintf('plotting...\n')
    %plot(x_hist, y_hist)
    %title('x and y traveled')
    %xlabel('x')
    %ylabel('y')
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