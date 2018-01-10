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

memoryName = 'MyMemory';

disp('*************************************');
disp('Running the test_CRCore');

% Start a timer
tic;
pause(0.1);
t = toc;

% Output result of timer
fprintf('t = %f\n', t);

% Open a shared memory object
mem = CRSharedMemory(memoryName, CR_MANAGER_SERVER);

% Create a vector of data
v = [0.1 0.4]';

% Add a signal
mem.addSignal('signal_1', v);

% Open a shared memory object as client
mem2 = CRSharedMemory(memoryName, CR_MANAGER_CLIENT);
	
dt = 0.1;
i = 0;
while i < 10
	tic;
	v = mem2.get('signal_1');
    v = v * 2;
    mem.set('signal_1', v);
	i = i + 1;
	fprintf('i = %i, signal = %f, %f\n', i, v(1), v(2));
	t = toc;
	pause(dt - t);
end

% Remove the signal
mem.removeSignal('signal_1');