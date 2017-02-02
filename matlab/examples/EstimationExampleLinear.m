%% EstimationExampleLinear
%
% Here we create an example robot in 2D to provide insight to the 
% kinematics methods accessible through the Manipulator class.
%
% This script is set up to run from inside the matlab/examples folder.
%
clear all
clc


%% Import the CoreRobotics Library
addpath ../
import CoreRobotics.*


%% Create a system to estimate
m  = 1;     % [kg]
k  = 5;     % [N/m]
b  = 2;     % [N-s/m]
a  = 5;     % [rad/s]
dt = 0.01;  % [s]

% Discretized system
A = eye(3) + [ 0,    1,    0;
              -k/m, -b/m,  1/m;
               0,    0,   -a].*dt;
B = [0; 1/m; 0].*dt;
G = [0,   0;
     1/m, 0;
     0,   a].*dt;
H = [1, 0, 0];
Q = diag([0.02, 1]);
R = 1e-6;


Motion = CoreRobotics.MotionModelLG(A,B,G,Q);
Sensor = CoreRobotics.SensorModelLG(H,R);

MotionSim = CoreRobotics.MotionModelLG(A,B,G,Q);
SensorSim = CoreRobotics.SensorModelLG(H,R);


%% Create Kalman Filter
KF = CoreRobotics.KalmanFilter(Motion,Sensor);


%% Simulate the state with some noise

t = 0:dt:5;
u = ones(size(t));
u(t<1) = 0;
w = 0.05*ones(size(t));
w(t<3) = 0;

% set up state
x = zeros(3,length(t));
y = zeros(1,length(t));
xEst = x;

% I.C.
MotionSim.state = x(:,1);
Motion.state = x(:,1);

for k = 1:length(t)
    x(:,k) = MotionSim.state;
    y(:,k) = SensorSim.SimulateMeasurement();
    MotionSim.state = MotionSim.SimulateMotion(u(k));
    SensorSim.state = MotionSim.state;
    
    % estimate
    xEst(:,k) = KF.Step(y(:,k),u(k));
end

figure(1),clf
subplot(121)
hold on; grid on;
plot(t,x)
plot(t,y,'.k')

subplot(122)
hold on; grid on;
plot(t,xEst)


% *************************************************************************
% Software License Agreement (BSD-3-Clause License)
% Copyright (c) 2017, CoreRobotics.
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are 
% met:
%
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%
%     * Neither the name of CoreRobotics nor the names of its contributors
%       may be used to endorse or promote products derived from this 
%       software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
% TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
% PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
% OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
% PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
% *************************************************************************
