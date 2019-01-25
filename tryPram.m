% Berk tryPram 2019/1/22

function tryPram()
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
       
        count = 0;
        state = 0;
        maxVel = 150*pi/180;
        leftVel = 0;
        rightVel = 0;
        dPos = zeros(3);
        dPosIntegral = zeros(3);
%         dAngle = zeros(3);
%         dAngleIntegral = zeros(3);
        proxState = zeros(6);
       

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
       
        vrep.simxSynchronousTrigger(clientID);
        vrep.simxGetObjectPosition(clientID,tar,tip,vrep.simx_opmode_streaming);
        vrep.simxGetObjectOrientation(clientID,tar,tip,vrep.simx_opmode_streaming);
        for i = 1:6
            vrep.simxReadProximitySensor(clientID,proxSensor(i),vrep.simx_opmode_streaming);
        end
        
        vrep.simxSynchronousTrigger(clientID);
        while toc<100

             [res,dPos] = vrep.simxGetObjectPosition(clientID,tar,tip,vrep.simx_opmode_buffer);
%              [res,dAngle] = vrep.simxGetObjectOrientation(clientID,tar,tip,vrep.simx_opmode_buffer);
             dPosIntegral = integrator(dPosIntegral,dPos,20);
             dist = sqrt(dPos(1)^2 + dPos(2)^2);
             theta = atan2(dPos(2),dPos(1));
%              dAngleIntegral = integrator(dAngleIntegral,dAngle,5);
             
             for i = 1:6
                 [res,proxState(i)] = vrep.simxReadProximitySensor(clientID,proxSensor(i),vrep.simx_opmode_buffer);
             end
             
             switch state
                 case 0
                     leftVel = 5*dist - 5*theta;
                     rightVel = 5*dist + 5*theta;
                     if dist < 0.1
                         state = 1;
                     end
                     
                 case 1
                     leftVel = 0;
                     rightVel =  0;
                     if dist > 0.1
                         state = 0;
                     end
             end
             vrep.simxSetJointTargetVelocity(clientID,leftMotor,leftVel,vrep.simx_opmode_oneshot);
             vrep.simxSetJointTargetVelocity(clientID,rightMotor,rightVel,vrep.simx_opmode_oneshot);
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
    for i = 1:3
        if accum(i) > bound
            accum(i) = bound;
        elseif accum(i) < -bound
            accum(i) = -bound;
        end
    end
end
