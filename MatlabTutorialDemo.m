function  MatlabTutorialDemo(serPort)

%==========================================================================
% Instructions:
% Each Segment 1-4 is an autonomous
% block of code that can be run indivudually
%==========================================================================

%==========================================================================
% 1. Basic Navigation
%==========================================================================
% SetFwdVelRadiusRoomba(serPort, 0.5, inf);     % Move Forward Full Speed
% SetFwdVelRadiusRoomba(serPort, -0.5, inf);    % Move Backward Full Speed
% turnAngle(serPort, 0.1, 90)                   % Turn Left
% turnAngle(serPort, 0.1, -90)                  % Turn Right
% SetFwdVelRadiusRoomba(serPort, 0, inf);       % Stop
%==========================================================================


%==========================================================================
% 2. Basic Navigation with Time Constraints
%==========================================================================    
% SetFwdVelRadiusRoomba(serPort, 0.5, inf);      % Move Forward
% pause(1)                                       % Pause for 1 second
% SetFwdVelRadiusRoomba(serPort, 0, inf);        % Stop
%==========================================================================

%==========================================================================
% 3. Read from Sensors
%==========================================================================
%[ BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort); % Read Bumpers
%display(BumpLeft)                              % Display Left Bumper Value
%display(BumpRight)                             % Display Right Bumper Value
%display(BumpFront)                             % Display Front Bumper Value
%WallSensor = WallSensorReadRoomba(serPort);    % Read Wall Sensor, Requires WallsSensorReadRoomba file    
%display(WallSensor)                            % Display WallSensor Value
%==========================================================================

%==========================================================================
% 4. While Loop with Maximum Time and Distance Sensor
%==========================================================================

% % Variable Declaration
% tStart= tic;                                        % Time limit marker
% maxDuration = 20;                                   % 20 seconds of max duration time    
% Initial_Distance = DistanceSensorRoomba(serPort);   % Get the Initial Distance
% Total_Distance = 0;                                 % Initialize Total Distance
%
% while toc(tStart) < maxDuration
%     [ BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort); % Read Bumpers
%     display(BumpLeft)                                % Display Left Bumper Value
%     display(BumpRight)                               % Display Right Bumper Value
%     display(BumpFront)                               % Display Front Bumper Value
%     WallSensor = WallSensorReadRoomba(serPort);      % Read Wall Sensor, Requires WallsSensorReadRoomba file    
%     display(WallSensor)                              % Display WallSensor Value
%
%     if(BumpRight)                                     % If the iRobot Create Bumped on the Right
%         turnAngle(serPort, 0.1, 90)                   % Turn Left 90 degrees
%         SetFwdVelRadiusRoomba(serPort, 0, 2);         % Stop
%     elseif(BumpLeft)                                  % Else if the iRobot Create Bumped on the Left
%         turnAngle(serPort, 0.1, -90)                  % Turn Right 90 degrees
%         SetFwdVelRadiusRoomba(serPort, 0, 2);         % Stop
%     elseif(BumpFront)                                 % Else if the iRobot Create Bumped on the Front
%         turnAngle(serPort, 0.1, -90)                  % Turn Right 90 degrees
%         SetFwdVelRadiusRoomba(serPort, 0, 2);         % Stop
%     else                                              % Else the iRobot hasn't bumped into anything
%         SetFwdVelRadiusRoomba(serPort, 0.1, 2);       % Move Forward
%         Total_Distance = Total_Distance + DistanceSensorRoomba(serPort);    % Update the Total_Distance covered so far
%         display(Total_Distance)                                             % Display the Total_Distance covered so far
%     end
% end  
% SetFwdVelRadiusRoomba(serPort, 0, 2);                                       % Stop the Robot
%==========================================================================

end