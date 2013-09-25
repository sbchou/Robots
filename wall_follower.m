% function [wall_sensor]   = AnalogWallSensorReadRoomba(serPort)
%   try
%     set(serPort,'timeout',0.01);
%     %Flush buffer
%     N = serPort.BytesAvailable();
%     while(N~=0) 
%       fread(serPort,N);
%       N = serPort.BytesAvailable();
%     end
%   catch
%   end

%   %% Get (142) Wall Reading(8) data fields
%   fwrite(serPort, [142 27]);
%   wall_sensor = fread(serPort, 1, 'uint16');
% end

%roomba moves ccwise
function dist = wall_follower(serPort)
  v= 0.2;
  w= 0;
  SetFwdVelAngVelCreate(serPort,v,w);

  distance_travelled = 0;
  bumped = false;
  prev_bump = false;

  while true
    bumped_side = bump_check(serPort);
    if bumped_side 
      if ~prev_bump
        %turn to the right without moving until right sensor is triggered
        right = false
        % stop
        SetFwdVelAngVelCreate(serPort, 0, 0); % stop
        while ~right
          turnAngle(r,0.1, -10) % turn right until sensor is triggered
          bumped = bump_check(serPort)
          right = bumped == 'right'
          pause(0.1)
        end
        SetFwdVelAngVelCreate(serPort, 0.2, 0); % continue forward
      else %no prev bump
        if bumped_side == 'right' || bumped_side == 'left'
          prev_bump = bumped_side;
          % continue straight because side is active 
          SetFwdVelAngVelCreate(serPort,0.2,0);
        else
          % front sensor has been triggered, turn towards the right
          right = false
          while ~right
            turnAngle(serPort, 0.1, 10)
            bumped = bump_check(serPort)
            right = bumped == 'right'
            pause(0.1)
          end
        end
      end
    else
      % no sensors triggered. if prev_bump is initialized, turn in the direction of the previous sensor and then go forward. If not, just continue.
        if prev_bump == 'right'
          turnAngle(serPort, 0.2, -10)
          SetFwdVelAngVelCreate(serPort, 0.2, 0); % continue forward
        elseif prev_bump == 'left'
          turnAngle(serPort, 0.2, 10)
          SetFwdVelAngVelCreate(serPort, 0.2, 0); % continue forward
        else
          SetFwdVelAngVelCreate(serPort, 0.2, 0); % continue forward
        end
    end 
    distance_travelled = distance_travelled + DistanceSensorRoomba(serPort);
    pause(0.1);
  end
  
  v = 0;
  w = 0;
  SetFwdVelAngVelCreate(serPort, v,w);  
  dist = distance_travelled;
end

function bumped_side = bump_check(serPort)
  [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
  if BumpRight 
    bumped_side = 'right'; 
  elseif BumpLeft
    bumped_side = 'left';
  elseif BumpFront
    bumped_side = 'front';
  else
    bumped_side = false;
  end
end
