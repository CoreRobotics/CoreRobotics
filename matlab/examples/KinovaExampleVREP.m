%% KinovaExampleVREP
%
% This example creates a manipulator object with the Kinova MICO v1
% modified DH parameters and interfaces with the VREP scene to send joint
% commands to the MICO.
%
% This script is set up to run from inside the +CoreRobotics/examples
% folder.
%
clear all
clc


%% Import the CoreRobotics Library
addpath ../
import CoreRobotics.*


%% Create the Robot

% DH parameters (modifed)
q1 = CoreRobotics.Math.DegToRad(-8.50e1);	% [rad] 
q2 = CoreRobotics.Math.DegToRad(+1.75e2);   % [rad] 
q3 = CoreRobotics.Math.DegToRad(+7.90e1);   % [rad] 
q4 = CoreRobotics.Math.DegToRad(-1.17e2);   % [rad] 
q5 = CoreRobotics.Math.DegToRad(+8.30e1);   % [rad] 
q6 = CoreRobotics.Math.DegToRad(+7.50e1);   % [rad] 

% lengths [m]
D1 = 0.2755;
D2 = 0.2900;
D3 = 0.1233;
D4 = 0.0741;
D5 = D4;
D6 = 0.1600;
e2 = 0.0070;

aa  = 30*pi/180;
sa  = sin(aa);
s2a = sin(2*aa);
d4b = D3 + sa/s2a*D4;
d5b = sa/s2a*(D4 + D5);
d6b = sa/s2a*D5 + D6;

% alpha(i-1) | r(i-1) | d(i) | theta(i)
DH = [ 0,    0,  D1,  q1;        % link 1
      -pi/2, 0,  0,   q2;        % 2
       0,    D2, e2,  q3;        % 3
      -pi/2, 0,  d4b, q4;        % 4
       2*aa, 0,  d5b, q5;        % 5
       2*aa, 0,  d6b, q6];       % 6

% define the offsets for conversion from DH to real world
% q(Mico) = qSign.*q(DH) + qOffset
qOffset = CoreRobotics.Math.DegToRad([180; -270; 90; 180; 180; 270]);
qSign   = [-1; 1; -1; -1; -1; -1];

% Create the Manipulator object and add RigidBody links
Mico = CoreRobotics.Manipulator;
for k = 1:6
    T = CoreRobotics.TransformationDH('FREE_VARIABLES','dh_theta');
    T.dh_alpha = DH(k,1);
    T.dh_r = DH(k,2);
    T.dh_d = DH(k,3);
    T.dh_theta = DH(k,4);
    Link = CoreRobotics.RigidBody('transform',T);
    Mico = Mico.AddLink(Link);
end




%% Connect to VREP
%
% Make sure the V-REP scene is running and that the following line of code
% is in the initialization portion of a non-threaded Lua child script
%
% simExtRemoteApiStart(19999)
%
% This command starts the remote API server, allowing MATLAB to connect.
%


% define the joints
jointName = {'Mico_joint1';
             'Mico_joint2';
             'Mico_joint3';
             'Mico_joint4';
             'Mico_joint5';
             'Mico_joint6'};
jointHandle = zeros(6,1);

toolName = 'MicoHand_attachPoint';
toolHandle = 0;


addpath ../external/vrep/

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
else
    vrep.delete(); % call the destructor!
    error('Failed connecting to remote API server');
end
    
% Now send some data to V-REP in a non-blocking fashion:
vrep.simxAddStatusbarMessage(clientID,...
    'CoreRobotics Connection to V-REP Initialized.',...
    vrep.simx_opmode_oneshot);


% Get the handles to the MICO joints
for i = 1:length(jointName)
    [result,jointHandle(i)]=vrep.simxGetObjectHandle(clientID,...
        jointName{i},vrep.simx_opmode_blocking);
    if (result~=vrep.simx_return_ok)
        disp('Could not find the specified joint handles.')
    end
end

% Get the handle to the MICO tool
[result,toolHandle]=vrep.simxGetObjectHandle(clientID,...
    toolName,vrep.simx_opmode_blocking);
if (result~=vrep.simx_return_ok)
    disp('Could not find the specified handle.')
end


% Define some commands to send to the Kinova
n = 1001;
u = [q1+linspace(0,pi/2,n);
     q2+linspace(0,pi/2,n);
     q3+linspace(0,pi/2,n);
     q4+linspace(0,pi/2,n);
     q5+linspace(0,pi/2,n);
     q6+linspace(0,pi/2,n)];
% u = repmat(linspace(0,2*pi,n),[6 1]);
q = zeros(6,n); % joint angles
y = zeros(6,n); % tool pose

%% main()
figure(1),clf
hold on; grid on; axis square
xlim([-0.5 0.5]), xlabel('x')
ylim([-0.5 0.5]), ylabel('y')
zlim([0 1]), zlabel('z')
view(3)


for k = 1:length(u)
    
    % Send the command
    for i = 1:length(jointName)
        posComd = u(i,k);
        result = vrep.simxSetJointPosition(clientID,jointHandle(i),...
            posComd,vrep.simx_opmode_oneshot);
    end
    
    % Run command through CoreRobotics Manipulator
    qDH = qOffset + qSign.*u(:,k);
    FK = Mico.GetForwardKinematics(qDH);
    cr_toolPosition = FK(:,end)';
    figure(1),cla
    plot3(FK(1,:),FK(2,:),FK(3,:),'.-k','MarkerSize',20,'LineWidth',3)
    drawnow
    
    % Return the position
    for i = 1:length(jointName)
        [result,posResp] = vrep.simxGetJointPosition(clientID,...
            jointHandle(i),vrep.simx_opmode_oneshot);
        q(i,k) = posResp;
    end
    
    % Return the tool position & orientation
    % vrep.sim_handle_parent
    [result,toolPosition] = vrep.simxGetObjectPosition(clientID,...
    	toolHandle,-1,vrep.simx_opmode_streaming);
    % V-REP euler angle convention = R_x(a)*R_y(b)*R_z(g)
    [result,toolOrientation] = vrep.simxGetObjectOrientation(clientID,...
    	toolHandle,-1,vrep.simx_opmode_streaming);
    
    % Display tool pose
    clc
    fprintf('VREP  = (%+5.3f, %+5.3f, %+5.3f) (%+5.3f, %+5.3f, %+5.3f)\n',...
        toolPosition(1),toolPosition(2),toolPosition(3),...
        toolOrientation(1),toolOrientation(2),toolOrientation(3));
    
    fprintf('CRLib = (%+5.3f, %+5.3f, %+5.3f)\n',...
        cr_toolPosition(1),cr_toolPosition(2),cr_toolPosition(3));
    
    vrep.simxGetPingTime(clientID);
    
end

%% Close
% Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID);

% Now close the connection to V-REP:
vrep.simxFinish(clientID);
vrep.delete(); % call the destructor!