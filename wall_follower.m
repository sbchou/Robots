function dist = wall_follower(serPort)
  v= 0.5;
  w= 0;
  SetFwdVelAngVelCreate(serPort,v,w);

  distance_travelled = 0;
  bumped = false;

  while ~bumped
    bumped = bump_check(serPort);
    distance_travelled = distance_travelled + DistanceSensorRoomba(serPort);
    pause(0.1)
  end
  v = 0
  w = 0
  SetFwdVelAngVelCreate(serPort, v,w);
  dist = distance_travelled;
end

function hit_wall = bump_check(serPort)
  [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
  hit_wall = BumpRight || BumpLeft || BumpFront;
end