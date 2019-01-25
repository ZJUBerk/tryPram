%% Configuration codes 
jointNum=2;    baseName='Base';
torqueLimit=[50,50];
jointName='Joint';  
displayOn=false;
torque=zeros(jointNum,1); qd=zeros(jointNum,1); intE=zeros(jointNum,1);
E=[]; Q=[];  DQ=[]; QD=[]; DQD=[]; TAU=[];

%% Connect to the Vrep
% 1. load api library
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
% 2. close all the potential link
vrep.simxFinish(-1);   
% 3. wait for connecting vrep, detect every 0.2s
while true
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    if clientID>-1 
        break;
    else
        pause(0.2);
        disp('please run the simulation on vrep...')
    end
end
disp('Connection success!')
% 4. set the simulation time step
tstep = 0.005;  % 5ms per simulation pass
vrep.simxSetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,tstep,vrep.simx_opmode_oneshot);
% 5. open the synchronous mode to control the objects in vrep
vrep.simxSynchronous(clientID,true);

%% Simulation Initialization
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
% get the handles
jointHandle=zeros(jointNum,1); 
for i=1:jointNum  % handles of the left and right arms
    [~,returnHandle]=vrep.simxGetObjectHandle(clientID,[jointName,int2str(i)],vrep.simx_opmode_blocking);
    jointHandle(i)=returnHandle;
end
[~,baseHandle]=vrep.simxGetObjectHandle(clientID,baseName,vrep.simx_opmode_blocking);  % reference coordinates of the arms'
vrep.simxSynchronousTrigger(clientID);
disp('Handles available!')
% first call to read the joints' configuration and end-effector pose, the mode is chosen to be simx_opmode_streaming
jointConfig=zeros(jointNum,1); 
for i=1:jointNum
    [~,jpos]=vrep.simxGetJointPosition(clientID,jointHandle(i),vrep.simx_opmode_streaming);
    jointConfig(i)=jpos;
end

for i=1:jointNum
    vrep.simxSetJointTargetVelocity(clientID,jointHandle(i),0.1,vrep.simx_opmode_oneshot);
    vrep.simxSetJointForce(clientID,jointHandle(i),0,vrep.simx_opmode_oneshot);
end
% get simulation time
currCmdTime=vrep.simxGetLastCmdTime(clientID); 
tipPosLast=tipPos;        
tipOrtLast=tipOrt;
lastCmdTime=currCmdTime;
jointConfigLast=jointConfig;

vrep.simxSynchronousTrigger(clientID);         % every calls this function, verp is triggered, 50ms by default

%% Simulation Start
t=0;  % start the simulation
while (vrep.simxGetConnectionId(clientID) ~= -1)  % vrep connection is still active
    % 0. time update
    currCmdTime=vrep.simxGetLastCmdTime(clientID);
    dt=(currCmdTime-lastCmdTime)/1000;              % simulation step, unit: s 
    % 1. read the joints' and tips' configuration (position and velocity)
    for i=1:jointNum
        [~,jpos]=vrep.simxGetJointPosition(clientID,jointHandle(i),vrep.simx_opmode_buffer);
        jointConfig(i)=jpos;
    end
    % using Finit Difference Method to get the velocity of joints' configuration (a simple version)
    q=jointConfig;       dq=(q-jointConfigLast)./dt;  % column vector
    % 2. display the acquisition data, or store them if plotting is needed.
    if displayOn==true
        disp('joints config:');       disp(q'.*180/pi);
        disp('joints dconfig:');      disp(dq'.*180/pi);
    end
    % 3. control alorithm can write down here
    % 1.pid controller
    %   3.1 feedback signal q,dq,c,dc, more importantly input the reference signal
    qd=[sin(t),cos(t)]';  dqd=[cos(t),-sin(t)]';
    qe=qd-q; dqe=dqd-dq;
    %   3.2 actuatation calculation
    Kp=[60,0;0,60];
    Kd=[2,0;0,2];
    Ki=[0.02 0;0 0.02];
    intE=intE+qe.*dt;
    torque=ddqd+Kp*qe+Kd*dqe+Ki*intE;

    %   3.3 set the torque in vrep way
    for i=1:jointNum
        if sign(torque(i))<0
            setVel=-9999; % set a trememdous large velocity for the screwy operation of the vrep torque control implementaion
            if torque(i)<-torqueLimit(i)
                setTu=-torqueLimit(i);
            else
                setTu=-torque(i);
            end
        else
            setVel=9999;
            if torque(i)>torqueLimit(i)
                setTu=torqueLimit(i);
            else
                setTu=torque(i);
            end
        end
        vrep.simxSetJointTargetVelocity(clientID,jointHandle(i),setVel,vrep.simx_opmode_oneshot);
        vrep.simxSetJointForce(clientID,jointHandle(i),setTu,vrep.simx_opmode_oneshot);
    end

    % data recording for plotting
    E=[E qe];    Q=[Q q];        QD=[QD qd];
    DQ=[DQ dq];  DQD=[DQD dqd];  TAU=[TAU torque];

    % 4. update vrep(the server side)
    tipPosLast=tipPos;       
    tipOrtLast=tipOrt;
    lastCmdTime=currCmdTime;
    jointConfigLast=jointConfig;    
    vrep.simxSynchronousTrigger(clientID);
    t=t+dt; % updata simulation time
end
vrep.simxFinish(-1);  % close the link
vrep.delete();        % destroy the 'vrep' class

%%
figure(1); hold off;
subplot(411); plot(E(1,:),'r'); hold on; plot(E(2,:),'b'); title('error')
subplot(412); plot(Q(1,:),'r'); hold on; plot(Q(2,:),'b');  plot(QD(1,:),'k'); hold on; plot(QD(2,:),'k'); title('trajectory')
subplot(413); plot(DQ(1,:),'r'); hold on; plot(DQ(2,:),'b');  plot(DQD(1,:),'k'); hold on; plot(DQD(2,:),'k'); title('velocity')
subplot(414); plot(TAU(1,:),'r'); hold on; plot(TAU(2,:),'b'); title('torque')
--------------------- 
作者：DianyeHuang 
来源：CSDN 
原文：https://blog.csdn.net/huangdianye/article/details/80628932 
版权声明：本文为博主原创文章，转载请附上博文链接！