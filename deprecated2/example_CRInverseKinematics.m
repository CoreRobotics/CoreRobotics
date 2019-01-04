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
disp('Demonstration of CRInverseKinematics.');

% Set the Euler convention we will use throughout the example
% Although CoreRobotics offers the flexibility to choose a
% different convention for each method, in general it is good
% to adopt the same convention throughout a problem for
% consistency.
convention = CR_EULER_MODE_XYZ;

% ------------------------------------------
% Create the robot

% Create several rigid body links
F0 = FrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G);
F1 = FrameEuler(1, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G);
F2 = FrameEuler(2, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G);
F3 = FrameEuler(1, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE);

Link0 = RigidBody(F0);
Link1 = RigidBody(F1);
Link2 = RigidBody(F2);
Link3 = RigidBody(F3);

% Create a new robot and add the links
MyRobot = Manipulator();

MyRobot.addLink(Link0);
MyRobot.addLink(Link1);
MyRobot.addLink(Link2);
attachLink = MyRobot.addLink(Link3);

% Create a tool frame and add to MyRobot
Tool = FrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE);
toolIndex = MyRobot.addTool(attachLink, Tool);

% ------------------------------------------
% Solve several invrese kinematics problems

% Set up an inverse kinematics object and attach the robot
ikSolver = InverseKinematics(MyRobot, toolIndex, convention);

% **********************
% CASE 1 : Solver should find a solution within default tolerance (1 mm),
% step size (1), and gain (0.1)
disp('---------------------------------------------');
disp('CASE 1: Use the default solver parameters.');

% Set the initial configuration of the robot
q0 = [0.1, -0.2, 0]';
MyRobot.setConfiguration(q0);

% Define a set point pose
p = [2.5, 0, 0, 0, 0, 0]';

% Define the solution array
qSolved = q0;

% Now solve the inverse kinematics for the point
tic;
result = ikSolver.solve(p, q0, qSolved);
et = toc;

if result == CR_RESULT_SUCCESS
	fprintf('Non-singular solution found in %f s!\n', et);
	disp(qSolved);

	% Now push the new joints through the robot to see if it worked
	MyRobot.setConfiguration(qSolved);
	fk = MyRobot.getForwardKinematics();

	disp('The forward kinematics for this solution are:');
	disp(fk);
else
	disp('The solution is singular.');
	disp(qSolved);
end

% **********************
% CASE 2:
disp('---------------------------------------------');
disp('CASE 2: Change the default maximum iteration.');

% Change the maximum iterations
ikSolver.setMaxIter(100);

% I.C.
MyRobot.setConfiguration(q0);

% Now solve the inverse kinematics for the point
tic;
result = ikSolver.solve(p, q0, qSolved);
et = toc;

if result == CR_RESULT_SUCCESS
	fprintf('Non-singular solution found in %f s!\n', et);
	disp(qSolved);

	% Now push the new joints through the robot to see if it worked
	MyRobot.setConfiguration(qSolved);
	fk = MyRobot.getForwardKinematics();

	disp('The forward kinematics for this solution are:');
	disp(fk);
else
	disp('The solution is singular.');
	disp(qSolved);
end

% **********************
% CASE 3: Change the tolerance and gain
disp('---------------------------------------------');
disp('CASE 3: Change the parameters.');

% Change the solver parameters
ikSolver.setMaxIter(100);
ikSolver.setStepSize(0.2);
ikSolver.setTolerance(0.0001);

% I.C.
MyRobot.setConfiguration(q0);

% Now solve the inverse kinematics for the point
tic;
result = ikSolver.solve(p, q0, qSolved);
et = toc;

if result == CR_RESULT_SUCCESS
	fprintf('Non-singular solution found in %f s!\n', et);
	disp(qSolved);

	% Now push the new joints through the robot to see if it worked
	MyRobot.setConfiguration(qSolved);
	fk = MyRobot.getForwardKinematics();

	disp('The forward kinematics for this solution are:');
	disp(fk);
else
	disp('The solution is singular.');
	disp(qSolved);
end

% **********************
% CASE 4: Assign a set point that is not reachable (singularity test)
disp('---------------------------------------------');
disp('CASE 4: Test a singular solution.');

% Assign a set point outside the robot reach
p = [5, 0, 0, 0, 0, 0]';

% I.C.
MyRobot.setConfiguration(q0);

% Now solve the inverse kinematics for the point
tic;
result = ikSolver.solve(p, q0, qSolved);
et = toc;

if result == CR_RESULT_SUCCESS
	fprintf('Non-singular solution found in %f s!\n', et);
	disp(qSolved);

	% Now push the new joints through the robot to see if it worked
	MyRobot.setConfiguration(qSolved);
	fk = MyRobot.getForwardKinematics();

	disp('The forward kinematics for this solution are:');
	disp(fk);
else
	disp('The solution is singular.');
	disp(qSolved);
end

% **********************
% CASE 5: Reduced pose vector
disp('---------------------------------------------');
disp('CASE 5: Reduced pose vector.');

% Assign a reduced set point
elems = poseElements(true, true, false, false, false, true);

pRed = [2.5, 0, 0]'; % (x, y, g)

% Change solver parameters
ikSolver.setStepSize(0.1);
ikSolver.setTolerance(0.001);

% I.C.
MyRobot.setConfiguration(q0);

% Now solve the inverse kinematics for the point
tic;
result = ikSolver.solve(pRed, elems, q0, qSolved);
et = toc;

if result == CR_RESULT_SUCCESS
	fprintf('Non-singular solution found in %f s!\n', et);
    disp(qSolved);

	% Now push the new joint through the robot to see if it worked
	MyRobot.setConfiguration(qSolved);
	fk = MyRobot.getForwardKinematics();

	disp('The forward kinematics for this solution are:');
	disp(fk);
else
	disp('The solution is singular.');
	disp(qSolved);
end

% **********************
% CASE 6: Single step convergance
disp('---------------------------------------------');
disp('CASE 6: Single step convergance.');

% Assign a set point
p = [2.5, 0, 0, 0, 0, 0]';

% Change solver parameters
ikSolver.setMaxIter(1);

% I.C.
MyRobot.setConfiguration(q0);

% Define a configuration
q = q0;

% Now solve the inverse kinematics for the point
for i = 1:100
	tic;
	result = ikSolver.solve(p, q, qSolved);
	et = toc;

	q = qSolved;

	if result == CR_RESULT_SUCCESS
        fprintf('Solution found in %f s!\n', et);

		% Now push the new joints through the robot to see if it worked
		MyRobot.setConfiguration(q);
		fk = MyRobot.getForwardKinematics();
	else
		disp('The solution is singular.');
    end
end

% -------------------------------------------------------------