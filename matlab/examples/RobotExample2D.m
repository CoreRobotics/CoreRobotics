%% RobotExample2D
%
% Here we create an example robot in 2D to provide insight to the
% kinematics methods accessible through the Manipulator class.
%
% This script is set up to run from inside the matlab/examples
% folder.
%
clear all
clc


%% Import the CoreRobotics Library
addpath ../
import CoreRobotics.*


%% Create the Robot

l1 = 0.5;   % [m] - link 1 length
l2 = 1.0;   % [m] - link 2 length
l3 = 0.4;   % [m] - link 3 length

% Define the transformations
T(1) = CoreRobotics.Frame2D('ang_a',0,'free_variables','ang_a');
T(2) = CoreRobotics.Frame2D('ang_a',-pi/2,'pos_x',l1,'free_variables','ang_a');
T(3) = CoreRobotics.Frame2D('ang_a',0,'pos_x',l2,'free_variables','ang_a');
T(4) = CoreRobotics.Frame2D('pos_x',l3);

% Create the Manipulator object and add RigidBody links
Robot = CoreRobotics.Manipulator;
for k = 1:length(T)
    Link = CoreRobotics.RigidBody('frame',T(k));
    Robot = Robot.AddLink(Link);
end


%% Run the robot through 

% Generate the free variable input (the order is because of how we added
% them to the Manipulator object)
u = [linspace(0,pi,101);            % T(1).ang_a
     linspace(-pi/2,pi/2,101);      % T(2).ang_a
     linspace(-pi,0,101)];          % T(3).ang_a

% Set up a plot.
figure(1),clf
hold on; grid on; axis equal;
xlabel('x'), ylabel('y')
title('Planar Robot with Jacobian vectors shown')
xlim([-2 2])
ylim([-2 2])

% Loop through the input vectors
for k = 1:length(u)
    
    % Get the FK from the Robot object
    [FK,Robot] = Robot.GetForwardKinematics(u(:,k));
    
    % Plot the FK point
    figure(1),cla
    plot(FK(1,:),FK(2,:),'.-k','MarkerSize',20,'LineWidth',3)
    
    % Now get the Jacobian for all links and plot the vectors to visualize
    % the effect of each transformation on the joints.
    J = Robot.GetJacobian(1);
    L = Robot.size_driven_links;
    for i = 1:Robot.size_links
        X = FK(1,i)*ones(1,L);
        Y = FK(2,i)*ones(1,L);
        U = J(1,:,i);
        V = J(2,:,i);
        quiver(X,Y,U,V)
    end
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