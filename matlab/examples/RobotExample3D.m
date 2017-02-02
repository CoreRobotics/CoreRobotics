%% RobotExample3D
%
% Here we create an example robot in 3D to provide insight to the
% kinematics methods accessible through the Manipulator class.
%
% This script is set up to run from inside the matlab/examples folder.
%
clear all
clc


%% Import the CoreRobotics Library
addpath ../
import CoreRobotics.*


%% Create the Robot

l1 = 1.0;   % [m] - link 1 length
l2 = 0.5;   % [m] - link 2 length

% Define the transformations
T(1) = CoreRobotics.Frame3D('convention','xyz','free_variables','ang_g');
T(2) = CoreRobotics.Frame3D('convention','xyz','free_variables','ang_a');
T(3) = CoreRobotics.Frame3D('convention','xyz','free_variables','ang_a','pos_y',l1);
T(4) = CoreRobotics.Frame3D('convention','xyz','pos_y',l2);

% Create the Manipulator object and add RigidBody links
Robot = CoreRobotics.Manipulator;
for k = 1:length(T)
    Link = CoreRobotics.RigidBody('frame',T(k));
    Robot = Robot.AddLink(Link);
end


%% Run the robot through 

% Generate the free variable input (the order is because of how we added
% them to the Manipulator object)
u = [linspace(-pi,0,101);           % T(1).ang_a
     linspace(pi/2,pi/4,101);       % T(2).ang_a
     linspace(-pi/2,-pi/4,101)];    % T(3).ang_a


% Set up a plot.
figure(1),clf
hold on; grid on; axis equal;
xlabel('x'), ylabel('y'), zlabel('z')
title('3D Robot')
xlim([-1.5 1.5])
ylim([-1.5 1.5])
zlim([0 2])
view(3)

% Loop through the input vectors
for k = 1:length(u)
    
    % Get the FK from the Robot object
    [FK,Robot] = Robot.GetForwardKinematics(u(:,k));
    
    % Plot the FK point
    figure(1),cla
    plot3(FK(1,:),FK(2,:),FK(3,:),'.-k','MarkerSize',20,'LineWidth',3)
    Robot.tool_frame.PlotFrame(gca,0.25);
    drawnow
end


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
