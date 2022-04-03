function gaussian (mean, variance)
-- Generate a sample from a Gaussian distribution

    return  math.sqrt(-2 * variance * math.log(math.random() + 0.00001)) *
            math.cos(2 * math.pi * math.random()) + mean
end


function getMaxMotorAngleFromTarget(posL, posR)

    -- How far are the left and right motors from their targets? Find the maximum
    maxAngle = 0
    if (speedBaseL > 0) then
        remaining = motorAngleTargetL - posL
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseL < 0) then
        remaining = posL - motorAngleTargetL
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseR > 0) then
        remaining = motorAngleTargetR - posR
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseR < 0) then
        remaining = posR - motorAngleTargetR
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end

    return maxAngle
end


function createRandomBumpyFloor()
    print ("Generating new random bumpy floor.")
    sim.setThreadAutomaticSwitch(false)

    -- Remove existing bumpy floor if there already is one
    if (heightField ~= nil) then
        sim.setObjectPosition(heightField, heightField, {0.05, 0, 0})
        return
    end
    --  Create random bumpy floor for robot to drive on
    floorSize = 5
    --heightFieldResolution = 0.3
    --heightFieldNoise = 0.00000005
    heightFieldResolution = 0.1
    heightFieldNoise = 0.0000008
    cellsPerSide = floorSize / heightFieldResolution
    cellHeights = {}
    for i=1,cellsPerSide*cellsPerSide,1 do
        table.insert(cellHeights, gaussian(0, heightFieldNoise))
    end
    heightField=sim.createHeightfieldShape(0, 0, cellsPerSide, cellsPerSide, floorSize, cellHeights)
    -- Make the floor invisible
    sim.setObjectInt32Param(heightField,10,0)
    sim.setThreadAutomaticSwitch(true)
end

function get_walls()
    -- Disable error reporting
    local savedState=sim.getInt32Param(sim.intparam_error_report_mode)
    sim.setInt32Param(sim.intparam_error_report_mode,0)
    local N = 1
    
    while true do
        local handle = sim.getObjectHandle("Wall"..tostring(N))
        if handle <= 0 then
            break
        end

        -- Read position and shape of wall
        -- Assume here that it is *thin* and oriented either along the x axis or y axis

        -- We can now get the propertries of these walls, e.g....
        local pos = sim.getObjectPosition(handle, -1)
        local res,minx = sim.getObjectFloatParameter(handle,15)
        local res,maxx = sim.getObjectFloatParameter(handle,18)
        local res,miny = sim.getObjectFloatParameter(handle,16)
        local res,maxy = sim.getObjectFloatParameter(handle,19)
    
        --print("Position of Wall " .. tostring(N) .. ": " .. tostring(pos[1]) .. "," .. tostring(pos[2]) .. "," .. tostring(pos[3]))
        --print("minmax", minx, maxx, miny, maxy)
 
        local Ax, Ay, Bx, By
        if (maxx - minx > maxy - miny) then
            print("Wall " ..tostring(N).. " along x axis")
            Ax = pos[1] + minx
            Ay = pos[2]
            Bx = pos[1] + maxx
            By = pos[2]
        else
            print("Wall " ..tostring(N).. " along y axis")
            Ax = pos[1]
            Ay = pos[2] + miny
            Bx = pos[1]
            By = pos[2] + maxy
        end
        print (Ax, Ay, Bx, By)

        walls[N] = {Ax, Ay, Bx, By}
        N = N + 1
    end
    -- enable error reporting
    sim.setInt32Param(sim.intparam_error_report_mode,savedState)

    return N - 1
end



function sysCall_init()
    -- This function is executed exactly once when the scene is initialised

    tt = sim.getSimulationTime()
    print("Init hello", tt)
    
    robotBase=sim.getObjectHandle(sim.handle_self) -- robot handle
    leftMotor=sim.getObjectHandle("leftMotor") -- Handle of the left motor
    rightMotor=sim.getObjectHandle("rightMotor") -- Handle of the right motor
    turretMotor=sim.getObjectHandle("turretMotor") -- Handle of the turret motor
    turretSensor=sim.getObjectHandle("turretSensor")

    -- Draw a red line between the waypoints for ease of measuring how
    -- well the robot performs
    lineSize=2
    maximumLines=9999
    red={1,0,0}
    drawingContainer = sim.addDrawingObject(sim.drawing_lines, lineSize, 0, -1, maximumLines, red)

    -- Create bumpy floor for robot to drive on
    createRandomBumpyFloor()

    N_WAYPOINTS = 26
    currentWaypoint = 0
    waypoints = {}
    waypoints[1] = {0.5,0}
    waypoints[2] = {1,0}
    waypoints[3] = {1,0.5}
    waypoints[4] = {1,1}
    waypoints[5] = {1,1.5}
    waypoints[6] = {1,2}
    waypoints[7] = {0.5,2}
    waypoints[8] = {0,2}
    waypoints[9] = {-0.5,2}
    waypoints[10] = {-1,2}
    waypoints[11] = {-1,1.5}
    waypoints[12] = {-1,1}
    waypoints[13] = {-1.5,1}
    waypoints[14] = {-2,1}
    waypoints[15] = {-2,0.5}
    waypoints[16] = {-2,0}
    waypoints[17] = {-2,-0.5}
    waypoints[18] = {-1.5,-1}
    waypoints[19] = {-1,-1.5}
    waypoints[20] = {-0.5,-1.5}
    waypoints[21] = {0,-1.5}
    waypoints[22] = {0.5,-1.5}
    waypoints[23] = {1,-1.5} 
    waypoints[24] = {1,-1}
    waypoints[25] = {0.5,-0.5}
    waypoints[26] = {0,0}
    
    for i=2, N_WAYPOINTS do
      sim.addDrawingObjectItem(drawingContainer, {waypoints[i-1][1],waypoints[i-1][2],0.1,waypoints[i][1],waypoints[i][2],0.1})
    end 
   
    -- Usual rotation rate for wheels (radians per second)
    speedBase = 5
    speedBaseL = 0
    speedBaseR = 0
    
    -- Which step are we in?
    -- 0 is a dummy value which is immediately completed
    stepCounter = 0
    stepCompletedFlag = false
    stepList = {}
    
    -- Dynamic step list table initialisation, depending on the waypoints above
    for i=1, N_WAYPOINTS do
        table.insert(stepList, {"set_waypoint", waypoints[i][1], waypoints[i][2]})
        table.insert(stepList, {"turn"})
        table.insert(stepList, {"stop"})
        table.insert(stepList, {"forward"})
        table.insert(stepList, {"stop"})
    end
    table.insert(stepList, {"repeat"})

    -- Create and initialise arrays for particles, and display them with dummies
    xArray = {}
    yArray = {}
    thetaArray = {}
    weightArray = {}
    dummyArray = {}
    N = 100
    for i=1, N do
        xArray[i] = 0
        yArray[i] = 0
        thetaArray[i] = 0
        weightArray[i] = 1/N
        dummyArray[i] = sim.createDummy(0.05)
        sim.setObjectPosition(dummyArray[i], -1, {0,0,0})
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,0})
    end
    
    -- Target positions for joints
    motorAngleTargetL = 0.0
    motorAngleTargetR = 0.0

     -- To calibrate
    motorAnglePerMetre = 24.8
    motorAnglePerRadian = 3.05
    
    e_variance = 0.001
    f_variance = 0.002
    g_variance = 0.002
    
    -- Data structure for walls
    walls = {}
    -- Fill it by parsing the scene in the GUI
    N_WALLS = get_walls()
    -- walls now is an array of arrays with the {Ax, Ay, Bx, By} wall coordinates
  
    sensorStandardDeviation = 0.1
    sensorVariance = sensorStandardDeviation^2
    noisyDistance = 0
end

function sysCall_sensing()
    
end


function calculateLikelihood(x ,y, theta, z)
    -- Declaration of default m and default closest wall for each particle 
    -- that calls this function
    m = math.maxinteger
    wall_index = -1
    
    -- For each possible closest wall ahead
    for i=1, N_WALLS do
        -- Calculation of m (the estimate of the distance to the wall based on our position)
        Ax = walls[i][1]
        Ay = walls[i][2]
        Bx = walls[i][3]
        By = walls[i][4]
        _num = (By - Ay) * (Ax - x) - (Bx - Ax)*(Ay - y) -- numerator of m division
        _den = (By - Ay) * math.cos(theta) - (Bx - Ax) * math.sin(theta) -- denominator of m division
        _m = _num / _den -- m calculation for particle
        
        -- Find point of collision
        x_collision = x + (_m * math.cos(theta))
        y_collision = y + (_m * math.sin(theta))
        
        -- AC vector 
        AC_vec = {Ax - x_collision , Ay - y_collision}
        -- AB vector
        AB_vec = {Ax - Bx, Ay - By}
        -- BC vector
        BC_vec = {Bx - x_collision, By - y_collision}
        -- BA vector
        BA_vec = {Bx - Ax, By - Ay}
        
        -- Dot product calculation for the four vectors
        dot_prod_1 = (AC_vec[1] * AB_vec[1]) + (AC_vec[2] * AB_vec[2])
        dot_prod_2 = (BC_vec[1] * BA_vec[1]) + (BC_vec[2] * BA_vec[2])
        
        -- Check if two dot products are positive (betwen wall endopoints)
        in_between_1 =  dot_prod_1 > 0
        in_between_2 =  dot_prod_2 > 0
        
        -- Take the minimum of m if the point is valid
        if _m < m and _m >= 0 and in_between_1 and in_between_2 then
            m = _m
            wall_index = i
        end
    end
    
    -- Returns the likelihood value, calculates it by measuring difference between m and z
    difference = z - m
    return math.exp(-math.pow(difference,2) / (2*sensorVariance)) 
end
    
function update_weights()
    -- Update all weights according to the likelihood function
    for i=1,N do
        weightArray[i] = weightArray[i] * calculateLikelihood(xArray[i], yArray[i], thetaArray[i], noisyDistance)
    end
end

function normalize_weights()
    -- Normalisation of particle weights
    sum = 0
    for i=1,N do 
        sum = sum + weightArray[i]
    end
    for i=1,N do
        weightArray[i] = weightArray[i] / sum
    end
end

function resample()
    -- Resampling of particles
    
    -- Cumulative array creation
    cumul_array = {}
    table.insert(cumul_array, weightArray[1])
    for i=2, N do 
        table.insert(cumul_array, cumul_array[i-1] + weightArray[i])
    end
    
    -- Declaration of temporary arrays x, y, theta, weights
    temp_xArray = {}
    temp_yArray = {}
    temp_thetaArray = {}
    temp_weightArray = {}
    for i=1, N do
        random_num = math.random() -- random number between 0 and 1
        for current=1, N do
            if random_num <= cumul_array[current] then
                table.insert(temp_xArray, xArray[current])
                table.insert(temp_yArray, yArray[current])
                table.insert(temp_thetaArray, thetaArray[current])
                table.insert(temp_weightArray, weightArray[current])
                sim.setObjectPosition(dummyArray[i], -1, {temp_xArray[i],temp_yArray[i],0})
                sim.setObjectOrientation(dummyArray[i], -1, {0,0, temp_thetaArray[i]})
                break
            end
        end
    end
    

    --Update the main arrays from the temporary arrays
    for index_current=1, N do
        xArray[index_current] = temp_xArray[index_current]
        yArray[index_current] = temp_yArray[index_current]
        thetaArray[index_current] = temp_thetaArray[index_current]
        weightArray[index_current] = temp_weightArray[index_current]
    end
    
    for i=1, N do
      weightArray[i] = 1/N
    end
end


function update_particle_cloud(step_type, new_step_amount)
    if step_type =="forward" then
        --print("Forward - Updating Particle cloud")
        for i=1, N do
            e = gaussian (0, e_variance)
            f = gaussian (0, f_variance)
            xArray[i] = xArray[i] + (new_step_amount + e) * math.cos(thetaArray[i])
            yArray[i] =  yArray[i] + (new_step_amount + e) * math.sin(thetaArray[i])
            thetaArray[i] = thetaArray[i] + f
            --sim.setObjectPosition(dummyArray[i], -1, {xArray[i],yArray[i],0})
            --sim.setObjectOrientation(dummyArray[i], -1, {0,0, thetaArray[i]})
        end
    elseif step_type =="turn" then
        --print("TURN - Updating Particle cloud")
        for i=1, N do
            g = gaussian (0, g_variance)
            thetaArray[i] = thetaArray[i] + new_step_amount + g
            --sim.setObjectOrientation(dummyArray[i], -1, {0, 0, thetaArray[i]})  
        end
    end
    
    if step_type =="forward" then
        for i=1, N do
            sim.setObjectPosition(dummyArray[i], -1, {xArray[i], yArray[i], 0})
            sim.setObjectOrientation(dummyArray[i], -1, {0, 0, thetaArray[i]})
        end
    elseif step_type =="turn" then
        for i=1, N do
            sim.setObjectOrientation(dummyArray[i], -1, {0, 0, thetaArray[i]})
        end
    end
end


function sysCall_actuation() 
    tt = sim.getSimulationTime()
    -- print("actuation hello", tt)
    
    -- Get and plot current angles of motor joints
    posL = sim.getJointPosition(leftMotor)
    posR = sim.getJointPosition(rightMotor)

    result, cleanDistance = sim.readProximitySensor(turretSensor)
    if (result>0) then
        -- sonar measurement
        noisyDistance = cleanDistance + gaussian(0.0, sensorVariance)
        --print ("Depth sensor reading ", noisyDistance)
    end
    
    
    -- Start new step?
    if (stepCompletedFlag == true or stepCounter == 0) then
        stepCounter = stepCounter + 1
        stepCompletedFlag = false


        newStepType = stepList[stepCounter][1]

        if (newStepType == "repeat") then
            -- Loop back to the first step
            stepCounter = 1
            newStepType = stepList[stepCounter][1]
            --print("New step:", stepCounter, newStepType)
        end
        if (newStepType == "forward") then
            -- Forward step: set new joint targets
            newStepAmount = forward_step_amount
            motorAngleTargetL = posL + newStepAmount * motorAnglePerMetre
            motorAngleTargetR = posR + newStepAmount * motorAnglePerMetre      
        elseif (newStepType == "turn") then
            -- Turn step: set new targets
            newStepAmountAngle = turn_step_amount
            
            -- Make the turn angle to be within the valid bounds (-360 < angle < 360)
            while newStepAmountAngle > math.rad(360) or newStepAmountAngle < -math.rad(360) do
                if newStepAmountAngle > math.rad (360) then
                    newStepAmountAngle = newStepAmountAngle - math.rad(360)
                elseif newStepAmountAngle < -math.rad(360) then
                   newStepAmountAngle = newStepAmountAngle + math.rad(360)
                end
            end
            
            -- Make the turn angle to be within the valid bounds (-180 < angle < 180)
            if (newStepAmountAngle > 0 and newStepAmountAngle > math.rad(180)) then
              newStepAmountAngle =  newStepAmountAngle - math.rad(360)
              -- print("angle turn set (1)", math.deg(newStepAmountAngle))
            elseif (newStepAmountAngle < 0 and newStepAmountAngle < - math.rad(180)) then
              newStepAmountAngle = newStepAmountAngle + math.rad(360)
              -- print("angle turn set (2)",math.deg( newStepAmountAngle))
            end
                
            motorAngleTargetL = posL - newStepAmountAngle * motorAnglePerRadian
            motorAngleTargetR = posR + newStepAmountAngle * motorAnglePerRadian
           
        elseif (newStepType == "stop") then
        
            --print ("Stopping!")
            
            -- Check if the last step was forward or turn
            if stepList[stepCounter-1][1] == "forward" then
                update_particle_cloud("forward", newStepAmount)
            elseif stepList[stepCounter-1][1] == "turn" then
                update_particle_cloud("turn", newStepAmountAngle)
            end

            -- At stop, we update the weights, normalize them and resample
            update_weights()
            normalize_weights()
            resample()

        elseif (newStepType == "set_waypoint") then
            --print("Setting a new way point")
            -- Estimating the position (from the average of the particle cloud)
            position_estimate = {}
            avg_x = 0
            avg_y = 0
            avg_theta = 0
            for i=1, N do
                avg_x = (avg_x +(xArray[i] * weightArray[i]))
                avg_y = (avg_y + (yArray[i] * weightArray[i]))
                avg_theta = (avg_theta + (thetaArray[i] * weightArray[i]))
            end
            
            position_estimate = {avg_x,avg_y,avg_theta}
            dx = stepList[stepCounter][2] - position_estimate[1]
            dy = stepList[stepCounter][3] - position_estimate[2]
            print({avg_x,avg_y,math.deg(position_estimate[3])})

            while position_estimate[3] > math.rad(360) or position_estimate[3] < -math.rad(360) do
                if position_estimate[3] > math.rad (360) then
                    position_estimate[3] = position_estimate[3] - math.rad(360)
                elseif position_estimate[3] < -math.rad(360) then
                   position_estimate[3] = position_estimate[3] + math.rad(360)
                end
            end
            print({avg_x,avg_y,math.deg(position_estimate[3])})
            print(math.deg((math.atan2(dy,dx))))
            -- Computing forward step amount (ie. the distance to the waypoint)
            forward_step_amount = math.sqrt(dx^2 + dy^2)
            -- Computing turn angle and whether it is a left or a right turn (ie. angle of the turn to face the waypoint)
            turn_step_amount = (math.atan2(dy,dx) - position_estimate[3])

            --print("Way point :", stepList[stepCounter][2], stepList[stepCounter][3])
            --print("forward_step_amount :", forward_step_amount)
            --print("")

            stepCompletedFlag = true

        end
    end


    -- Handle current ongoing step
    stepType = stepList[stepCounter][1]

    if (stepType == "turn") then
        if (newStepAmountAngle >= 0) then
            speedBaseL = -speedBase
            speedBaseR = speedBase
        else
            speedBaseL = speedBase
            speedBaseR = -speedBase
        end
        motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- Slow down when close
        if (motorAngleFromTarget < 3) then
            speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        if (motorAngleFromTarget == 0) then
            stepCompletedFlag = true
        end
    elseif (stepType == "forward") then
        speedBaseL = speedBase
        speedBaseR = speedBase
        motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- Slow down when close
        if (motorAngleFromTarget < 3) then
            speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        if (motorAngleFromTarget == 0) then
            stepCompletedFlag = true
        end
    elseif (stepType == "stop") then
        speedBaseL = 0
        speedBaseR = 0

        -- Check to see if the robot is stationary to within a small threshold
        linearVelocity,angularVelocity=sim.getVelocity(robotBase)
        vLin = math.sqrt(linearVelocity[1]^2 + linearVelocity[2]^2 + linearVelocity[3]^2)
        vAng = math.sqrt(angularVelocity[1]^2 + angularVelocity[2]^2 + angularVelocity[3]^2)
        --print ("stop", linearVelocity, vLin, vAng)
    
        if (vLin < 0.001 and vAng < 0.01) then
            stepCompletedFlag = true
        end
    end

    -- Set the motor velocities for the current step
    sim.setJointTargetVelocity(leftMotor,speedBaseL)
    sim.setJointTargetVelocity(rightMotor,speedBaseR)        
   
end

function sysCall_cleanup()
    --simUI.destroy(ui)
end 
