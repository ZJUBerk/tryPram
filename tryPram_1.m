% Berk tryPram_1 2019/1/25

function tryPram_1()
    clear;clc;
    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
        % start the simulation:

        %%
        %initialization
        tic
        vrep.simxSynchronous(clientID,true);
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
       
        kP1 = 5;
        kI1 = 0.2;
        kD1 = 0;
        kP2 = 10; %default = 7
        kI2 = 0;
        kD2 = 0.2;
        %Braitenberg weights
        leftBraitWeight = 1.2*[0.45, -0.2, 0.4, -0.15, 0.3, -0.05, 0.3, -0.5];
        rightBraitWeight = 1.2*[-0.2, 0.45, -0.15, 0.4, -0.05, 0.3, -0.5, 0.3];
        
        count = 0;
        state = 0;
        turnFlag = zeros(2,1);
        dt = 0.01;  %time step = 10ms
        simTime = 0;
        diamWheel = 0.1;
        unitVel = 2/diamWheel;  %1/(pi*diamWheel)*(2*pi)
        maxVel = 2*unitVel; %indoor mode maxVel = 1.5m/s; outdoor mode maxVel = 2m/s.
        maxAcc = 10*unitVel*dt;
        keepDist = 1.5; 
        avoidDist = 0.5;
        pastLeftVel = 0;
        pastRightVel = 0;
        leftVel = 0;
        rightVel = 0;
%         tarLineVel = zeros(1,3);
%         tarAngVel = zeros(1,3);
        tipLineVel = zeros(1,3);
        tipAngVel = zeros(1,3);        
        realPos = zeros(1,3);
%         posIntegral = zeros(1,3);
%         posDiff = zeros(1,3);
        pastDist = 0;
        dist = 0;
        distIntegral = 0;
        distDiff = 0;
        pastTheta = 0;
        theta = 0;
        thetaIntegral = 0;
        thetaDiff = 0;
        proxNum = 8;
        proxState = zeros(proxNum,1);
        proxCoord = zeros(proxNum,3);
        proxDist = zeros(proxNum,1);
        pastProxDist = zeros(proxNum,1);
        pastIndex = ones(proxNum,1);
        
        tempWalkVel = zeros(100,1);
        pastWalkVel = 0;
        walkVel = 0;
        
        wallFlag = 0;
        
        Q = 0.0001*eye(2);
        R = 0.001*eye(2);
        A = eye(2);
        H = eye(2);
        n = size(Q);
        m = size(R);
        z = zeros(2,1);
        xhat = zeros(2,1);
        xhatminus = zeros(2,1);
        P = [1 0;0 1];
        Pminus = zeros(n);
        K = zeros(n(1),m(1));
        I = eye(n);
        pos = zeros(2,1);

        [res,leftMotor] = vrep.simxGetObjectHandle(clientID,'leftMotor',vrep.simx_opmode_blocking);       
        [res,rightMotor] = vrep.simxGetObjectHandle(clientID,'rightMotor',vrep.simx_opmode_blocking);  
        [res,tip] = vrep.simxGetObjectHandle(clientID,'tip',vrep.simx_opmode_blocking);
        [res,tar] = vrep.simxGetObjectHandle(clientID,'tar',vrep.simx_opmode_blocking);
        [res,proxSensor(1)] = vrep.simxGetObjectHandle(clientID,'sensor_f_l',vrep.simx_opmode_blocking);
        [res,proxSensor(2)] = vrep.simxGetObjectHandle(clientID,'sensor_f_r',vrep.simx_opmode_blocking);  
        [res,proxSensor(3)] = vrep.simxGetObjectHandle(clientID,'sensor_l_f',vrep.simx_opmode_blocking);
        [res,proxSensor(4)] = vrep.simxGetObjectHandle(clientID,'sensor_r_f',vrep.simx_opmode_blocking);
        [res,proxSensor(5)] = vrep.simxGetObjectHandle(clientID,'sensor_l_b',vrep.simx_opmode_blocking);
        [res,proxSensor(6)] = vrep.simxGetObjectHandle(clientID,'sensor_r_b',vrep.simx_opmode_blocking);
        [res,proxSensor(7)] = vrep.simxGetObjectHandle(clientID,'sensor_d_l',vrep.simx_opmode_blocking);
        [res,proxSensor(8)] = vrep.simxGetObjectHandle(clientID,'sensor_d_r',vrep.simx_opmode_blocking);
       
        vrep.simxSynchronousTrigger(clientID);

        vrep.simxGetObjectPosition(clientID,tar,tip,vrep.simx_opmode_streaming);
        vrep.simxGetObjectPosition(clientID,tar,-1,vrep.simx_opmode_streaming);
        vrep.simxGetObjectPosition(clientID,tip,-1,vrep.simx_opmode_streaming);

        for i = 1:proxNum
            vrep.simxReadProximitySensor(clientID,proxSensor(i),vrep.simx_opmode_streaming);
        end
        vrep.simxGetObjectVelocity(clientID,tar,vrep.simx_opmode_streaming);
        vrep.simxGetObjectVelocity(clientID,tip,vrep.simx_opmode_streaming);
        
        vrep.simxSynchronousTrigger(clientID);
        %%
        %start loop
        while toc < 2000
            simTime = simTime + dt;
            pastDist = dist; 
            pastTheta = theta;
            [res,realPos] = vrep.simxGetObjectPosition(clientID,tar,tip,vrep.simx_opmode_buffer);
            z = realPos(1:2)' + sqrtm(R)*randn(2,1);   %add sensor noise
            %call Kalman filter
            xhatminus = A*xhat+sqrtm(Q);
            Pminus = A*P*A'+Q;
            K = Pminus*H'*inv( H*Pminus*H'+R );
            xhat = xhatminus+K*(z-H*xhatminus);
            P = (I-K*H)*Pminus;
            pos(1) = xhat(1);
            pos(2) = xhat(2);
%             pos(1) = z(1); %without Kalman
%             pos(2) = z(2); %without Kalman

%             plot(simTime,realPos(1),'ro');hold on
%             plot(simTime,z(1),'b+');
%             plot(simTime,pos(1),'g*');
            
%             [res,tarLineVel,tarAngVel] = vrep.simxGetObjectVelocity(clientID,tar,vrep.simx_opmode_buffer)
%             [res,tipLineVel,tipAngVel] = vrep.simxGetObjectVelocity(clientID,tip,vrep.simx_opmode_buffer);

            dist = sqrt(pos(1)^2 + pos(2)^2);
            distIntegral = integrator(distIntegral,dist-keepDist,maxVel);
            distDiff = derivator(pastDist,dist,dt,maxVel);
            theta = atan2(pos(2),pos(1));
            thetaIntegral = integrator(thetaIntegral,theta,maxVel);
            thetaDiff = derivator(pastTheta,theta,dt,maxVel);

            [walkVel,tempWalkVel,pastWalkVel] = getWalkVel(tempWalkVel,pastWalkVel,dist,pastDist,dt,(leftVel+rightVel)/2/unitVel);
            
            for i = 1:proxNum
                [res,proxState(i),proxCoord(i,:)] = vrep.simxReadProximitySensor(clientID,proxSensor(i),vrep.simx_opmode_buffer);   
            end
            if proxState(7) ~= 0
                proxState(7) = 0;
            else
                proxState(7) = 1;
            end
            if proxState(8) ~= 0
                proxState(8) = 0;
            else
                proxState(8) = 1;
            end

            vrep.simxSetObjectPosition(clientID,tar,-1,[6*cos(0.2*simTime)+0.06*cos(4*pi*simTime);9*sin(0.1*simTime+1)+0.09*sin(4*pi*simTime+1);0.2],vrep.simx_opmode_oneshot);
%             vrep.simxSetObjectPosition(clientID,tar,-1,[3;10-simTime+0.02*cos(4*pi*simTime);0.2],vrep.simx_opmode_oneshot);
%             vrep.simxSetObjectPosition(clientID,tar,-1,[6.6*cos(0.2*simTime)+0.066*cos(4*pi*simTime);9.9*sin(0.1*simTime+1)+0.099*sin(4*pi*simTime+1);0.2],vrep.simx_opmode_oneshot);

            switch state
                case 0

                    if proxState == [0;0;0;0;0;0;0;0]
%                         leftVel = walkVel*unitVel + kP1*(dist - keepDist) + kI1*distIntegral + kD1*distDiff  - kP2*theta*(pastLeftVel+pastRightVel)^2/dist - kI2*thetaIntegral - kD2*thetaDiff;
%                         rightVel = walkVel*unitVel + kP1*(dist - keepDist) + kI1*distIntegral + kD1*distDiff + kP2*theta*(pastLeftVel+pastRightVel)^2/dist + kI2*thetaIntegral - kD2*thetaDiff;
                        leftVel = walkVel*unitVel + kP1*(dist - keepDist) + kI1*distIntegral + kD1*distDiff  - kP2*theta - kI2*thetaIntegral - kD2*thetaDiff;
                        rightVel = walkVel*unitVel + kP1*(dist - keepDist) + kI1*distIntegral + kD1*distDiff + kP2*theta + kI2*thetaIntegral - kD2*thetaDiff;
                        [leftVel,rightVel] = velLimit(pastLeftVel,pastRightVel,leftVel,rightVel,maxVel,maxAcc);
                    elseif proxState(1) == 1 && proxState(2) == 1
                        disp('hesitate!');
                        if theta > 0 
                            turnFlag(2) = 1;
                            leftVel = -0.5*unitVel;
                            rightVel = 0.1 * unitVel;
                        else
                            turnFlag(2) = -1;
                            leftVel = 0.1 * unitVel;
                            rightVel = -0.5*unitVel;
                        end
                        [leftVel,rightVel] = velLimit(pastLeftVel,pastRightVel,leftVel,rightVel,0.5*maxVel,maxAcc);
                        distIntegral = 0;
                        thetaIntegral = 0;
                        state = 3;
                    else
                        for i = 1:2
                            if proxState(i) == 0
                                proxDist(i) = avoidDist;
                            else
                                proxDist(i) = 0.5*proxCoord(i,3);
                            end
                            if proxDist(i) > pastProxDist(i)
                                pastIndex = 8;
                            else
                                pastIndex = 1;
                            end
                        end
                        for i = 3:6
                            if proxState(i) == 0
                                proxDist(i) = avoidDist;
                            else
                                proxDist(i) = 1*proxCoord(i,3);
                            end
                            if proxDist(i) > pastProxDist(i)
                                pastIndex = 8;
                            else
                                pastIndex = 1;
                            end
                        end
                        for i = 7:8
                            if proxState(i) == 0
                                proxDist(i) = avoidDist;
                            else
                                proxDist(i) = 0;
                            end
                        end
                        if proxState == [0;0;1;0;0;0;0;0] & theta > 0
                            wallFlag = wallFlag + 1;
                        elseif proxState == [0;0;0;1;0;0;0;0] & theta < 0
                            wallFlag = wallFlag - 1;
                        end
                        if abs(wallFlag)>10
                            state = 4;
                        end
                        
                        pastProxDist = proxDist;
%                         leftVel = pastLeftVel + maxAcc*leftBraitWeight*((1-(proxDist/avoidDist)).^pastIndex);
%                         rightVel = pastRightVel + maxAcc*rightBraitWeight*((1-(proxDist/avoidDist)).^pastIndex);
                        leftVel = walkVel+pastLeftVel + maxAcc*leftBraitWeight*((1-(proxDist/avoidDist)).^pastIndex);
                        rightVel = walkVel+pastRightVel + maxAcc*rightBraitWeight*((1-(proxDist/avoidDist)).^pastIndex);
                        if proxState(7) == 1 || proxState(8) == 1
                            [leftVel,rightVel] = velLimit(pastLeftVel,pastRightVel,leftVel,rightVel,0.25*maxVel,maxAcc);
                        else
                            [leftVel,rightVel] = velLimit(pastLeftVel,pastRightVel,leftVel,rightVel,0.5*maxVel,maxAcc);
                        end
                        distIntegral = 0;
                        thetaIntegral = 0;
                    end
                    if abs(dist - keepDist) < 0.1 && walkVel == 0
                        state = 1;
                        distIntegral = 0;
                    end
                 
                case 1
                    if proxState == [0;0;0;0;0;0;0;0]
                        leftVel = -kP2*theta - kI2*thetaIntegral - kD2*thetaDiff;
                        rightVel = kP2*theta + kI2*thetaIntegral - kD2*thetaDiff;
                    else
                        leftVel = 0;
                        rightVel = 0;

                    end
                    [leftVel,rightVel] = velLimit(pastLeftVel,pastRightVel,leftVel,rightVel,0.5*maxVel,maxAcc);
                    distIntegral = 0;
                    if abs(dist - keepDist) >= 0.2
                        state = 0;
                    elseif abs(dist - keepDist) < 0.2 && abs(theta) < 0.05
                        state = 2;
                        thetaIntegral = 0;
                    end
                 
                case 2
                    leftVel = 0;
                    rightVel =  0;   
                    distIntegral = 0;
                    thetaIntegral = 0;
                    if abs(dist - keepDist) >= 0.2 || abs(theta) >= 0.05
                        state = 0;
                    end
                    [leftVel,rightVel] = velLimit(pastLeftVel,pastRightVel,leftVel,rightVel,0.5*maxVel,maxAcc);
                case 3
                    if turnFlag(2) > 0
                        if proxState(3) == 1 || proxState(5) == 1
                            turnFlag(2) = -1;
                            leftVel = 0.2*unitVel;
                            rightVel = -0.25*unitVel;
                        else
                            leftVel = -0.25*unitVel;
                            rightVel = 0.2*unitVel;
                        end
                    else
                        if proxState(4) == 1 || proxState(6) == 1
                            turnFlag(2) = 1;
                            leftVel = -0.25*unitVel;
                            rightVel = 0.2*unitVel;
                        else
                            leftVel = 0.2*unitVel;
                            rightVel = -0.25*unitVel;
                        end
                    end
                    [leftVel,rightVel] = velLimit(pastLeftVel,pastRightVel,leftVel,rightVel,0.5*maxVel,maxAcc);
                    disp('still hesitate!');
                    turnFlag(1) = turnFlag(1) +1;
                    distIntegral = 0;
                    thetaIntegral = 0;
                    if proxState(1:2) ~= [1;1]
                        turnFlag(1) = turnFlag(1)+150;
                    end
                    if turnFlag(1) > 1000
                        turnFlag(1) = 0;
                        state = 0;
                    end
                case 4  %wallFollow
                    if wallFlag > 0
                        leftVel = walkVel*unitVel + kP1*(dist - keepDist) + kI1*distIntegral + kD1*distDiff - 5*(proxCoord(3,3) - 0.4);
                        rightVel = walkVel*unitVel + kP1*(dist - keepDist) + kI1*distIntegral + kD1*distDiff + 5*(proxCoord(3,3) - 0.4);
                        if sum(proxState == [0;0;1;0;0;0;0;0]) ~= 8 || theta < 0
                            state = 0;
                            wallFlag = 0;
                        end
                    elseif wallFlag < 0
                        leftVel = walkVel*unitVel + kP1*(dist - keepDist) + kI1*distIntegral + kD1*distDiff + 5*(proxCoord(4,3) - 0.4);
                        rightVel = walkVel*unitVel + kP1*(dist - keepDist) + kI1*distIntegral + kD1*distDiff - 5*(proxCoord(4,3) - 0.4);
                        if sum(proxState == [0;0;0;1;0;0;0;0]) ~= 8 || theta > 0
                            state = 0;
                            wallFlag = 0;
                        end
                    disp('wall follow');
                    end
                    [leftVel,rightVel] = velLimit(pastLeftVel,pastRightVel,leftVel,rightVel,maxVel,maxAcc);
                    
                     
 
            %                      break
            end
            vrep.simxSetJointTargetVelocity(clientID,leftMotor,leftVel,vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,rightMotor,rightVel,vrep.simx_opmode_oneshot);

            pastLeftVel = leftVel;
            pastRightVel = rightVel;
            count=count+1;
%             pause(0.01);
            vrep.simxSynchronousTrigger(clientID);
                 
             
        end
    
        
        %%
        fprintf('cycles: %d\n',count);
        % stop the simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor! 
    disp('Program ended');
end
    
function matureData = integrator(accum, rawData ,bound)
    matureData = accum + rawData;
    for i = 1:length(matureData)
        if matureData(i) > bound
            matureData(i) = bound;
        elseif matureData(i) < -bound
            matureData(i) = -bound;
        end
    end
end

function resultData = derivator(pastData, presentData ,timeStep,bound)
    resultData = (presentData-pastData)/timeStep;
    for i = 1:length(pastData)
        if resultData(i) > bound
            resultData(i) = bound;
        elseif resultData(i) < 0
            resultData(i) = 0;
        end
    end
end

%motor speed limitation
function [leftVel,rightVel] = velLimit(pastLeftVel,pastRightVel,tempLeftVel,tempRightVel,maxVel,maxAcc)
%     if tempLeftVel-pastLeftVel > maxAcc
%         tempLeftVel = pastLeftVel + maxAcc;
%         disp('lal');
%     elseif tempLeftVel-pastLeftVel < -maxAcc
%         tempLeftVel = pastLeftVel - maxAcc;
%         disp('lal');
%     end
%     if tempRightVel-pastRightVel > maxAcc
%         tempRightVel = pastRightVel + maxAcc;
%         disp('ral');
%     elseif tempRightVel-pastRightVel < -maxAcc
%         tempRightVel = pastRightVel - maxAcc;
%         disp('ral');
%     end
    if tempLeftVel == 0 && tempRightVel == 0
        leftVel = 0;
        rightVel = 0;
    else
        vel1 = max(tempLeftVel,tempRightVel);
        vel2 = min(tempLeftVel,tempRightVel);
        if vel1 > maxVel && abs(vel2) < vel1
            vel2 = maxVel*vel2/vel1;
            vel1 = maxVel;
        elseif vel2 < -maxVel && abs(vel2) >= vel1
            vel1 = maxVel*vel1/abs(vel2);
            vel2 = -maxVel;
        end
        if tempLeftVel >= tempRightVel
            leftVel = vel1;
            rightVel = vel2;
        else
            leftVel = vel2;
            rightVel = vel1;
        end
    end
    if leftVel-pastLeftVel > maxAcc
        leftVel = pastLeftVel + maxAcc;
%         disp('lal');
    elseif leftVel-pastLeftVel < -maxAcc
        leftVel = pastLeftVel - maxAcc;
%         disp('lal');
    end
    if rightVel-pastRightVel > maxAcc
        rightVel = pastRightVel + maxAcc;
%         disp('ral');
    elseif rightVel-pastRightVel < -maxAcc
        rightVel = pastRightVel - maxAcc;
%         disp('ral');
    end
end

%limit-range recursive average filtering to get walk speed
%First order low pass filtering
function [walkVel,tempWalkVel,pastWalkVel] = getWalkVel(tempWalkVel,pastWalkVel,dist,pastDist,timeStep,pramVel)
    for i = 1:9
        tempWalkVel(i) = tempWalkVel(i+1);
    end
    tempWalkVel(10) = (dist - pastDist) / timeStep + pramVel;
    if abs(tempWalkVel(10)-pastWalkVel) > 10
        tempWalkVel(10) = tempWalkVel(9);
    end   
    aveWalkVel = sum(tempWalkVel)/10;
    walkVel = 0.99*pastWalkVel + 0.01*aveWalkVel;
    
    if walkVel < 0.01
        walkVel = 0;
    end
    pastWalkVel = walkVel;
end 
       
        