%=====================================================================
%
% Software License Agreement (BSD-3-Clause License)
% Copyright (c) 2017, CoreRobotics.
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions
% are met:
% 
% * Redistributions of source code must retain the above copyright
% notice, this list of conditions and the following disclaimer.
% 
% * Redistributions in binary form must reproduce the above copyright
% notice, this list of conditions and the following disclaimer in the
% documentation and/or other materials provided with the distribution.
% 
% * Neither the name of CoreRobotics nor the names of its contributors
% may be used to endorse or promote products derived from this
% software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
% FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
% COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
% ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
% 
% \project CoreRobotics Project
% \url     www.corerobotics.org
% \author  Cameron Devine
% 
%
%=====================================================================

% Import CoreRobotics
import CoreRobotics.*

disp('*************************************');
disp('Demonstration of CRManipulator.');

% Create a new robot
MyRobot = CRManipulator();

% Create a couple of Euler frames and rigid body links
F0 = CRFrameEuler();
F1 = CRFrameEuler();
F2 = CRFrameEuler();
Link0 = CRRigidBody();
Link1 = CRRigidBody();
Link2 = CRRigidBody();

% Set info for Link 0 and add to MyRobot
F0.setFreeVariable(CR_EULER_FREE_ANG_G);
F0.setMode(CR_EULER_MODE_XYZ);
F0.setPositionAndOrientation(0, 0, 0.5, 0, 0, 0);
Link0.setFrame(F0);
MyRobot.addLink(Link0);

% Set info for Link 1 and add to MyRobot
F1.setFreeVariable(CR_EULER_FREE_ANG_G);
F1.setMode(CR_EULER_MODE_XYZ);
F1.setPositionAndOrientation(1, 0, 0, 0, 0, 0);
Link1.setFrame(F1);
MyRobot.addLink(Link1);

% Set info for Link 2 and add to MyRobot
F2.setFreeVariable(CR_EULER_FREE_NONE);
F2.setMode(CR_EULER_MODE_XYZ);
F2.setPositionAndOrientation(2, 0, 0, 0, 0, 0);
Link2.setFrame(F2);
linkIndex = MyRobot.addLink(Link2);

% Create a tool frame and add to MyRobot
Tool = CRFrameEuler();
Tool.setMode(CR_EULER_MODE_XYZ);
Tool.setPositionAndOrientation(0, 0, 0, 0, 0, 0);
toolIndex = MyRobot.addTool(linkIndex, Tool);

% ------------------------------------------
% Try out some of the methods available to the manipulator

% Get the configuration values
dof = MyRobot.getDegreesOfFreedom();
jointAngles = MyRobot.getConfiguration();
fprintf('MyRobot has %i DOF, with joint angles (rad) =\n', dof);
disp(jointAngles);

% Now get the Forward Kinematics and Jacobian
FwdKin = MyRobot.getForwardKinematics();
Jacobian = MyRobot.jacobian(toolIndex, CR_EULER_MODE_XYZ);
disp('Forward Kinematics =');
disp(FwdKin);
disp('Jacobian =');
disp(Jacobian);

% Now set a new robot configuration and get the FK and jacobian
jointAngles = [pi / 4; -pi / 2];
disp('Set joint angles (rad) ='); 
disp(jointAngles);
MyRobot.setConfiguration(jointAngles);
FwdKin = MyRobot.getForwardKinematics();
Jacobian = MyRobot.jacobian(toolIndex, CR_EULER_MODE_XYZ);
disp('Forward Kinematics =');
disp(FwdKin);
disp('Jacobian =');
disp(Jacobian);

% Now get the transformation to the tool for the current configuration
toolFrame = CRFrame();
MyRobot.getToolFrame(toolIndex, toolFrame);
T = toolFrame.getTransformToParent();
disp('MyRobot tool has a transformation of');
disp(T);

% Get the jacobian for only (x, y, g)
elems = poseElements(true, true, false, false, false, true);
Jred = MyRobot.jacobian(toolIndex, CR_EULER_MODE_XYZ, elems);
disp('MyRobot Jacobian (reduced) is');
disp(Jred);

% Get the tool pose for only (x, y, g)
pose = MyRobot.getToolPose(toolIndex, CR_EULER_MODE_XYZ, elems);
disp('MyRobot pose (reduced) is');
disp(pose);

% ------------------------------------------