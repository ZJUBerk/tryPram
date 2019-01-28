% Berk tryPram1 2019/1/25

function tryPram1()
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
        tic
        vrep.simxSynchronous(clientID,true);
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
       
        P1 = 20;
        I1 = 1;
        D1 = 0.5;
        P2 = 20;
        I2 = 0;
        D2 = 0;
        
        count = 0;
        state = 0;
        dt = 0.05;  %time step = 50ms
        simTime = 0;
        diamWheel = 0.1;
        unitVel = 2/diamWheel;  %1/(pi*diamWheel)*(2*pi)
        maxVel = 2*unitVel;
        keepDist = 1;
        leftVel = 0;
        rightVel = 0;
        tarLineVel = zeros(3,1);
        tarAngVel = zeros(3,1);
        tipLineVel = zeros(3,1);
        tipAngVel = zeros(3,1);
        
        pos = zeros(3);
%         posIntegral = zeros(3);
%         posDiff = zeros(3);
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
       

        [res,leftMotor] = vrep.simxGetObjectHandle(clientID,'leftMotor',vrep.simx_opmode_blocking);       
        [res,rightMotor] = vrep.simxGetObjectHandle(clientID,'rightMotor',vrep.simx_opmode_blocking);  
        [res,tip] = vrep.simxGetObjectHandle(clientID,'tip',vrep.simx_opmode_blocking);
        [res,tar] = vrep.simxGetObjectHandle(clientID,'tar',vrep.simx_opmode_blocking);
        [res,obstacle_1] = vrep.simxGetObjectHandle(clientID,'cylinder_1',vrep.simx_opmode_blocking);
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
        vrep.simxGetObjectPosition(clientID,obstacle_1,-1,vrep.simx_opmode_streaming);
        vrep.simxGetObjectVelocity(clientID,tar,vrep.simx_opmode_streaming);
        vrep.simxGetObjectVelocity(clientID,tip,vrep.simx_opmode_streaming);
       
        vrep.simxSynchronousTrigger(clientID);
        [res,obstacle_1] = vrep.simxGetObjectPosition(clientID,obstacle_1,-1,vrep.simx_opmode_buffer);
        figure(1)
        plot(obstacle_1(1),obstacle_1(2),'o','color','g'); hold on
        
        for i = 1:proxNum
            vrep.simxReadProximitySensor(clientID,proxSensor(i),vrep.simx_opmode_streaming);
        end
        
        vrep.simxSynchronousTrigger(clientID);
        while toc < 100
             simTime = simTime + dt;
             pastDist = dist; 
             pastTheta = theta;
             
             [res,pos] = vrep.simxGetObjectPosition(clientID,tar,tip,vrep.simx_opmode_buffer);
             [res,absTarPos] = vrep.simxGetObjectPosition(clientID,tar,-1,vrep.simx_opmode_buffer);
             [res,absTipPos] = vrep.simxGetObjectPosition(clientID,tip,-1,vrep.simx_opmode_buffer);
             [res,tarLineVel,tarAngVel] = vrep.simxGetObjectVelocity(clientID,tar,vrep.simx_opmode_buffer);
             [res,tipLineVel,tipAngVel] = vrep.simxGetObjectVelocity(clientID,tip,vrep.simx_opmode_buffer);
             figure(1)
%              plot(simTime,(tipLineVel(1)^2+tipLineVel(2)^2)^0.5,'x');hold on
             plot(absTarPos(1),absTarPos(2),'x','color','r'); hold on
             plot(absTipPos(1),absTipPos(2),'o','color','b');
%              posIntegral = integrator(posIntegral,pos,maxVel);
%              posDiff = derivator(pastPos,pos,dt,maxVel);
             dist = sqrt(pos(1)^2 + pos(2)^2);
             distIntegral = integrator(distIntegral,dist-keepDist,maxVel);
             distDiff = derivator(pastDist,dist,dt,maxVel);
             theta = atan2(pos(2),pos(1));
             thetaIntegral = integrator(thetaIntegral,theta,maxVel);
             thetaDiff = derivator(pastTheta,theta,dt,maxVel);
             
             for i = 1:proxNum
                 [res,proxState(i)] = vrep.simxReadProximitySensor(clientID,proxSensor(i),vrep.simx_opmode_buffer);
             end
             
             
             switch state
                 case 0
                     leftVel =P1*(dist - keepDist) + I1*distIntegral + D1*distDiff  - P2*theta - I2*thetaIntegral - D2*thetaDiff;
                     rightVel = P1*(dist - keepDist) + I1*distIntegral + D1*distDiff + P2*theta + I2*thetaIntegral - D2*thetaDiff;
                     
                     %%
                     % simple obstacle avoidance
                     if proxState(1) && proxState(2)
                         if (proxState(3)||proxState(5)) && (~(proxState(4)||proxState(6)))
                             leftVel = unitVel;
                             rightVel = -unitVel;
                         elseif (proxState(4)||proxState(6)) && (~(proxState(3)||proxState(5)))
                             leftVel =-unitVel;
                             rightVel = unitVel;
                         elseif (proxState(3)||proxState(5)) && (proxState(4)||proxState(6))
                             disp('can not move,wall');
                             break;  
                         else
                             if theta > 0
                                 leftVel =-unitVel;
                                 rightVel = unitVel;
                             else
                                 leftVel = unitVel;
                                 rightVel = -unitVel;
                             end
                         end
                     elseif proxState(1) && ~proxState(2)
                         leftVel = unitVel;
                         rightVel = -unitVel;
                     elseif ~proxState(1) && proxState(2)
                         leftVel =-unitVel;
                         rightVel = unitVel;
                     else
                         if proxState(3)||proxState(5)
                             leftVel = unitVel;
                             rightVel = -unitVel;
                             if proxState(7) && ~proxState(8)
                                 disp('can not move,r h');
                                 break;
                             end
                         elseif proxState(4)||proxState(6)
                             leftVel =-unitVel;
                             rightVel = unitVel;
                             if ~proxState(7) && proxState(8)
                                 disp('can not move,l h');
                                 break;
                             end
                         end
                     end
                     if ~proxState(7) && proxState(8)
                         leftVel = 0.5*unitVel;
                         rightVel = -unitVel;
                         disp('g');
                     elseif proxState(7) && ~proxState(8)
                         leftVel = -unitVel;
                         rightVel = 0.5*unitVel;
                         disp('f');
                     elseif ~proxState(7) && ~proxState(8)
                         disp('can not move, h');
                         break;
                     end
                     %%
                     [leftVel,rightVel] = velLimit(leftVel,rightVel,maxVel);
                     if abs(dist - keepDist) < 0.1 && abs(theta) >= 0.05
                         state = 1;
                         distIntegral = 0;
                     elseif abs(dist - keepDist) < 0.1 && abs(theta) < 0.05
                         state = 2;
                         distIntegral = 0;
                         thetaIntegral = 0;
                     end
                     
                 case 1
                     leftVel = -P2*theta - I2*thetaIntegral - D2*thetaDiff;
                     rightVel = P2*theta + I2*thetaIntegral - D2*thetaDiff;
                     [leftVel,rightVel] = velLimit(leftVel,rightVel,maxVel);
                     distIntegral = 0;
                     if abs(dist - keepDist) >= 0.1
                         state = 0;
                     elseif abs(dist - keepDist) < 0.1 && abs(theta) < 0.05
                         state = 2;
                         thetaIntegral = 0;
                     end
                     
                 case 2
                     leftVel = 0;
                     rightVel =  0;
                     distIntegral = 0;
                     thetaIntegral = 0;
                     if abs(dist - keepDist) >= 0.1 || abs(theta) >= 0.05
                         state = 0;
                     end
%                      break
             end
             vrep.simxSetJointTargetVelocity(clientID,leftMotor,leftVel,vrep.simx_opmode_oneshot);
             vrep.simxSetJointTargetVelocity(clientID,rightMotor,rightVel,vrep.simx_opmode_oneshot);
%              vrep.simxSetJointTargetVelocity(clientID,leftMotor,-unitVel,vrep.simx_opmode_oneshot);
%              vrep.simxSetJointTargetVelocity(clientID,rightMotor,-unitVel,vrep.simx_opmode_oneshot);
             count=count+1;
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

function [leftVel,rightVel] = velLimit(tempLeftVel,tempRightVel,maxVel)
    if tempLeftVel == 0 && tempRightVel == 0
        leftVel = 0 && rightVel == 0;
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
end
        
        
        
        