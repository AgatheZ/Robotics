-- Generate a sample from a Gaussian distribution
function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) *
            math.cos(2 * math.pi * math.random()) + mean
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



-- This function is executed exactly once when the scene is initialised
function sysCall_init()

    tt = sim.getSimulationTime()
    print("Init hello", tt)
          
    robotBase=sim.getObjectHandle(sim.handle_self) -- robot handle
    leftMotor=sim.getObjectHandle("leftMotor") -- Handle of the left motor
    rightMotor=sim.getObjectHandle("rightMotor") -- Handle of the right motor
    turretMotor=sim.getObjectHandle("turretMotor") -- Handle of the turret motor
    turretSensor=sim.getObjectHandle("turretSensor")
 
    -- Create bumpy floor for robot to drive on
    createRandomBumpyFloor()

   
    -- Usual rotation rate for wheels (radians per second)
    speedBase = 5
    speedBaseL = 0
    speedBaseR = 0
    
    -- Which step are we in?
    -- 0 is a dummy value which is immediately completed
    stepCounter = 0
    stepCompletedFlag = false
    stepList = {}
        

    stepList[1] = {"set_waypoint",-1,-0.5}
    stepList[2] = {"turn"}
    stepList[3] = {"stop"}
    stepList[4] = {"forward"}
    stepList[5] = {"stop"}
    
    stepList[6] = {"set_waypoint",1,1}
    stepList[7] = {"turn"}
    stepList[8] = {"stop"}
    stepList[9] = {"forward"}
    stepList[10] = {"stop"}
    
    stepList[11] = {"set_waypoint",0,1}
    stepList[12] = {"turn"}
    stepList[13] = {"stop"}
    stepList[14] = {"forward"}
    stepList[15] = {"stop"}
    
    stepList[16] = {"set_waypoint",0,0}
    stepList[17] = {"turn"}
    stepList[18] = {"stop"}
    stepList[19] = {"forward"}
    stepList[20] = {"stop"}
    stepList[21] = {"repeat"}

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
    g_variance = 0.003
    noisyDistance = 0
end

function sysCall_sensing()
    
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

function sysCall_actuation() 
    tt = sim.getSimulationTime()
    -- print("actuation hello", tt)
    -- Get and plot current angles of motor joints
    posL = sim.getJointPosition(leftMotor)
    posR = sim.getJointPosition(rightMotor)
    


    -- Start new step?
    if (stepCompletedFlag == true or stepCounter == 0) then
        stepCounter = stepCounter + 1
        stepCompletedFlag = false

        newStepType = stepList[stepCounter][1]

        if (newStepType == "repeat") then
            -- Loop back to the first step
            stepCounter = 1
            newStepType = stepList[stepCounter][1]
        end

        print("New step:", stepCounter, newStepType)
 
        if (newStepType == "forward") then
            -- Forward step: set new joint targets
            newStepAmount = forward_step_amount
            motorAngleTargetL = posL + newStepAmount * motorAnglePerMetre
            motorAngleTargetR = posR + newStepAmount * motorAnglePerMetre
            -- Update paricle cloud (from last turn motion)
            for i=1, N do
                e = gaussian (0, e_variance)
                f = gaussian (0, f_variance)
                xArray[i] = xArray[i] + (newStepAmount + e) * math.cos(thetaArray[i])
                yArray[i] =  yArray[i] + (newStepAmount + e) * math.sin(thetaArray[i])
                thetaArray[i] = thetaArray[i] + math.rad(f)
                weightArray[i] = 1/N
                sim.setObjectPosition(dummyArray[i], -1, {xArray[i],yArray[i],0})
                sim.setObjectOrientation(dummyArray[i], -1, {0,0, thetaArray[i]})
            end             
        elseif (newStepType == "turn") then
            -- Turn step: set new targets
            newStepAmountAngle = turn_step_amount
            print("before angle turn set", math.deg(newStepAmountAngle))

            if (newStepAmountAngle > 0 and newStepAmountAngle > math.rad(180)) then
              newStepAmountAngle =  newStepAmountAngle - math.rad(360)
            elseif (newStepAmountAngle < 0 and newStepAmountAngle < - math.rad(180)) then
              newStepAmountAngle = newStepAmountAngle + math.rad(360) 
            end
            print("angle turn set", math.deg(newStepAmountAngle))
            
            motorAngleTargetL = posL - newStepAmountAngle * motorAnglePerRadian
            motorAngleTargetR = posR + newStepAmountAngle * motorAnglePerRadian
            
            -- Update particle cloud (from last forward motion)
            for i=1, N do
                g = gaussian (0, g_variance)
                thetaArray[i] = thetaArray[i] + newStepAmountAngle + g
                weightArray[i] = 1/N
                sim.setObjectOrientation(dummyArray[i], -1, {0, 0, thetaArray[i]})
            end
        elseif (newStepType == "stop") then
            print ("Stopping!")

        elseif (newStepType == "set_waypoint") then
            print("Setting a new way point")
            -- Estimating the position (from the average of the particle cloud)
            position_estimate = {0, 0, 0}
            for i=1, N do
                position_estimate[1] = position_estimate[1] + xArray[i] * weightArray[i]
                position_estimate[2] = position_estimate[2] + yArray[i] * weightArray[i]
                position_estimate[3] = position_estimate[3] + thetaArray[i] * weightArray[i]
            end
            dx = stepList[stepCounter][2] - position_estimate[1]
            dy = stepList[stepCounter][3] - position_estimate[2]
            
            -- Computing forward step amount (ie. the distance to the waypoint)
            forward_step_amount = math.sqrt(dx^2 + dy^2)
            
            -- Computing turn angle and whether it is a left or a right turn (ie. angle of the turn to face the waypoint)
            turn_step_amount = (math.atan2(dy,dx) - position_estimate[3])
            
            print("Way point :", stepList[stepCounter][2], stepList[stepCounter][3])
            print("forward_step_amount :", forward_step_amount)
            print("turn_step_amount", turn_step_amount)
            
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
