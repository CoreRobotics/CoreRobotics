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
disp('Demonstration of CRKalmanFilter.');

% Initialize matricies
A = [1 1; 0 1];
B = [0.5; 1];
C = [1 0];
Q = 0.1 * eye(2);
R = 0.3;
x = [500; 0];
Sigma0 = 0.01 * eye(2);
u = -9.81;

disp('---------------------------------------------');
disp('CASE 1: Discrete time Kalman filter.');

% Initialize the Kalman filter
kalman = CRKalmanFilter(A, B, C, Q, R, x, Sigma0);

for i = 0:9
	% Take a noisy measurement
	measurement = C * x + R * randn();
	% Step the system forward
	x = A * x + B * u + Q * randn(2, 1);
	kalman.step(u, measurement);
	% Print the results
	fprintf('At i = %i\n', i);
	fprintf('The true state is %f %f\n', x);
	fprintf('And the estimated state is %f %f\n', kalman.getState);
	disp('With covariance');
	disp(kalman.getCovariance());
end

disp('---------------------------------------------');
disp('CASE 2: Continuous time Kalman filter.');

% Test the system as continuous
x = [500; 0];
dt = 0.1;

kalman = CRKalmanFilter(A, B, C, Q, R, x, Sigma0, dt);

for i = 1:9
	% Create a noisy measurement
	measurement = C * x + R * randn();
	% Step the system forward
	xdot = A * x + B * u + Q * randn(2, 1);
	x = x + dt * xdot;
	kalman.step(u, measurement);
	% Print the results
	fprintf('At t = %f\n', i * dt);
	fprintf('The true state is %f %f\n', x);
	fprintf('And the estimated state is %f %f\n', kalman.getState);
	disp('With covariance');
	disp(kalman.getCovariance());
end
