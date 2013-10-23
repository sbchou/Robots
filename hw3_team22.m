% HW3 - Team 22
% Jason Ravel - sdm2140
%
function hw3_Team22(serPort)


    tic; 

  %variables
  
 
    diam_irobot = .35;
    time_out = 30; 
    speed_of_turn = .1;
    forward_speed = .25;
    it_is_free = 1; 
    UNEXPLORED= 0;
    Collision = 2; 
    hp_tresh = .3;

    
    wall_sensor_boolean = false; 
    try 
        AnalogWallSensorReadRoomba(serPort);
        speed_of_turn = 0.05;
        time_out = 120;
        wall_sensor_boolean = true;
        forward_speed = 0.05;
    catch err
    end
    

    origFig = gcf;
    posPlot = axes('Parent', figure());
    hold(posPlot);

    angle = 0; 
    
    hit_pos =0; 
    
    dist_from_hit = 0;

    roomPlot = axes('Parent', figure());
    figure(origFig);
   
    sensors = ReadSensors();

    
    heading = 0;
    
    the_pos = [0; 0];
        
    the_grid = [0]; 

   minimum_x = 0;
    minimum_y = 0;
       
    forwarding = 2;
    FOLLOWING_Collision=3; 
    FOLLOW_Collision = 0;
    turning = 1;

    lastNewPointTime = toc;

    state = forwarding;
    
    
    MainLoop();
    


    function MainLoop
        Drive(forward_speed, forward_speed); 
        
        turnAngle = 0; 
        turned = 0; 

%if diff in elapsed time and time last point observed > timeout, stop
        while(time_out >= toc - lastNewPointTime)       
            UpdateOdometry();
 
            switch (state)
                case FOLLOW_Collision
                    dist_from_hit = norm(the_pos - hit_pos);
                    if(dist_from_hit > hp_tresh)
                        state = FOLLOWING_Collision;
                        
                    elseif(sensors.Bump)
                        Drive(speed_of_turn, -speed_of_turn);
                    elseif(~sensors.Wall)
                        Drive(0, speed_of_turn);
                    else
                        Drive(forward_speed, forward_speed);
                    end
                    
                    UpdateGrid(Collision); 
                case FOLLOWING_Collision
                    dist_from_hit = norm(the_pos - hit_pos);
                    if(dist_from_hit < hp_tresh)
                        state = turning;
                        Drive(speed_of_turn, -speed_of_turn);
                        turnAngle = GenRandTurnAngle(90, 135);
                        turned = 0; 

                    elseif(sensors.Bump)
                        Drive(speed_of_turn, -speed_of_turn);
                    elseif(~sensors.Wall)
                        Drive(0, speed_of_turn);
                    else
                        Drive(forward_speed, forward_speed);
                    end
                    
                    UpdateGrid(Collision); 
                case turning
                    turned = turned + abs(sensors.Angle) / pi() * 180; 
                    if (turned > turnAngle)
                        Drive(forward_speed, forward_speed); 
                        state = forwarding; 
                    end
                case forwarding
                    if (sensors.Bump)
                        hit_pos = the_pos;
                        currentPoint = GetCurrentCell(); 
                        normCurrPoint = normPoint(currentPoint); 
                        x_idx = normCurrPoint(1);
                        y_idx = normCurrPoint(2); 
                        [x_size, y_size] = size(the_grid); 
                        if((x_idx * y_idx ~= 0) && (x_idx <= x_size && y_idx <= y_size) && ...
                            the_grid(x_idx, y_idx) == Collision)
                        
                            state = turning;
                            Drive(speed_of_turn, -speed_of_turn);
                            turnAngle = GenRandTurnAngle(0, 360);
                            turned = 0; 
                        else 
                            state = FOLLOW_Collision; 
                        end
                    else
                        UpdateGrid(it_is_free);             
                    end
            end 
            pause(0.1);
        end
        Stop()
    end


  
    function currentCell = GetCurrentCell()
        currentCell = floor(the_pos ./ diam_irobot);
    end

    function Drive(leftSpeed, rightSpeed)
        SetDriveWheelsCreate(serPort, leftSpeed, rightSpeed);
    end

    function turnAngle = GenRandTurnAngle(minAngle, maxAngle)
        turnAngle = minAngle + rand() * (maxAngle - minAngle); 
    end 

    function Stop()
        Drive(0,0); 
    end




    function axis = NormalizeAxis(len_axis, min_axis)
        axis = [0:len_axis-1];
        axis = axis + min_axis;
        axis = axis .* diam_irobot; 
    end


    function normalizedPoint = normPoint(cell)
        normalizedPoint = (cell - [minimum_x; minimum_y])  + [1; 1]; 
    end


    
    
    
    function g = PadTopRight(g)
        [len_x, len_y] = size(g);
                       
        g = [ g; ones(1,len_y) * UNEXPLORED];
        g = [g ones(len_x+1, 1) * UNEXPLORED];
    end


	function PlotRoom() 
        
        % Orient the grid correctly
        tmp_grid = rot90(the_grid);
        [len_x, len_y] = size(the_grid);
        
        %Pad axis by 1 so that pcolor actually shows the end of it.
        x = NormalizeAxis(len_x+1, minimum_x);
        y = NormalizeAxis(len_y+1, minimum_y);
        
        [X, Y] = meshgrid(x,y);
        
        %This is for show. If you don't this the top and right get cut off
        %in the final plot
        tmp_grid = PadTopRight(tmp_grid);
        Y = flipud(Y); 
        
        %Color map for 
        colormap([1 1 1; .5 1 .5; 1 0 0]);
        pcolor(roomPlot, X, Y, tmp_grid)
    end

	function UpdateOdometry()
	        lastHeading = heading;
	        lastthe_pos = the_pos;

	        sensors = ReadSensors();

	        angle = sensors.Angle;
	        dist = sensors.Dist;
	        if(angle == 0)
	            % Robot traveled in a straight line.
	            the_pos = the_pos + [cos(heading); sin(heading)] .* dist;
	        else
	            % arc
	            heading = mod(heading + angle, 2 * pi());
	            turnRadius = dist / angle;
	            the_pos = [0; -turnRadius];
	            the_pos = [cos(angle), -sin(angle); sin(angle), cos(angle)] * the_pos;
	            the_pos = the_pos + [0; turnRadius];
	            the_pos = [cos(lastHeading), -sin(lastHeading); ...
	                        sin(lastHeading), cos(lastHeading)] * the_pos;
	            the_pos = the_pos + lastthe_pos;
	        end



	        % plot the robot's path (from its point of view)
	         if(~isequal(the_pos, lastthe_pos))
	             plot(posPlot, [lastthe_pos(1); the_pos(1)], ...
	                           [lastthe_pos(2); the_pos(2)], 'b');
	         end
	    end

   function UpdateGrid(cellVal)
        currPoint = GetCurrentCell();
        [size_x, size_y] = size(the_grid);
        
        % Increase the size of the "world" if need be 
        if (currPoint(1) < minimum_x)
            minimum_x = currPoint(1);
            the_grid = [ones(1,size_y)*UNEXPLORED; the_grid]; 
            [size_x, size_y] = size(the_grid);
        end
        
        if (currPoint(1) - minimum_x >= size_x) 
            the_grid = [the_grid; ones(1, size_y)*UNEXPLORED]; 
            [size_x, size_y] = size(the_grid);
        end
                
        if (currPoint(2) < minimum_y)
            minimum_y = currPoint(2);
            the_grid = [ones(size_x, 1)*UNEXPLORED];
            [size_x, size_y] = size(the_grid);
        end        
        
        if (currPoint(2) - minimum_y >= size_y)
            the_grid = [the_grid ones(size_x, 1)*UNEXPLORED];
        end
        
        normCurrPoint = normPoint(currPoint); 
        x_idx = normCurrPoint(1);
        y_idx = normCurrPoint(2); 
        
        if (the_grid(x_idx, y_idx) == UNEXPLORED)
            fprintf('Raw(%d, %d) -> Logical(%d, %d) -- Min(%d, %d)\n', ...
                currPoint(1), currPoint(2), x_idx, y_idx, minimum_x, minimum_y);
            
            lastNewPointTime = toc;         
            the_grid(x_idx, y_idx) = cellVal;   
            PlotRoom()
        else if (the_grid(x_idx, y_idx) == it_is_free)
            if (cellVal == Collision)
                lastNewPointTime = toc; 
            end
            
            the_grid(x_idx, y_idx) = cellVal;
            end
            PlotRoom()
        end
     end

   


    function sensors = ReadSensors()
        
        sensors.Wall = (wall_sensor_boolean && CheckSensor(AnalogWallSensorReadRoomba(serPort)) >= 1) ... 
                    || CheckSensor(WallSensorReadRoomba(serPort));
      
        
        [sensors.BumpRight, sensors.BumpLeft, wdr, wdl, wdc, sensors.BumpFront] ...
            = BumpsWheelDropsSensorsRoomba(serPort);
                
        sensors.Bump = CheckSensor(sensors.BumpRight) ... 
                    || CheckSensor(sensors.BumpLeft) ... 
                    || CheckSensor(sensors.BumpFront);
                
        sensors.Angle = CheckSensor(AngleSensorRoomba(serPort));
        
        sensors.Dist = CheckSensor(DistanceSensorRoomba(serPort));
        
    end


    function sensorVal = CheckSensor(val)
        if (isnan(val))
            sensorVal = 0; 
            return 
        end
        
        sensorVal = val;
    end
end