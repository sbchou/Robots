%roomba moves ccwise
function dist = wall_follower(serPort)
  v= 0.2;
  w= 0;
  SetFwdVelAngVelCreate(serPort,v,w);

  distance_travelled = 0;
  bumped = false;
  t_start = tic;
  max_time = 120;
  prev_bump = false;

  while toc(t_start) < max_time
    bumped_side = bump_check(serPort);
    
    if bumped_side 
      if ~prev_bump
        %turn to the right without moving until right sensor is triggered
        right = false
        while ~right
          turnAngle(r,0.1, -10)
          [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster BumpFront] = BumpsWheelDropsSensorsRoomba(serPort)
          right = BumpRight
          pause(0.1)
        end
      else %no prev bump
        if bumped_side == 'right' || bumped_side == 'left'
          prev_bump = bumped_side;
          % continue straight because side is active 
          travelDist(serPort, 0.5, 0.1)
        else
          % front sensor has been triggered, turn away from the side that you were traveling along before
          right = false        
          while ~right
            turnAngle(serPort,0.1, 10)
            [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster BumpFront] = BumpsWheelDropsSensorsRoomba(serPort)
            right = BumpRight
            pause(0.1)
          end
        end
      end
    else
      % no sensors triggered. if prev_bump is initialized, turn in the direction of the previous sensor and then go forward. If not, just continue.
        if prev_bump == 'right'
          turnAngle(serPort, 0.2, -10)
        else
          turnAngle(serPort, 0.2, 10)
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
