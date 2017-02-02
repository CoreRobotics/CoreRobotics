% Script for testing library code.  This script does not test for
% correctness, but rather tests for the ability of classes to perform 
% methods as expected.  Please run this before pushing code to make sure it
% still runs properly.
%
% If you add new classes or new public/static methods, please be sure to 
% include them in this test script.

%%
addpath ../
import CoreRobotics.*
disp('Running CoreRobotics testLib utility...')
disp('***************************************')
k = 0;

%% CoreRobotics.Math
try out = CoreRobotics.Math.DegToRad(90); catch ME; k=k+1;disp('Math.DegToRad failed'); end
try out = CoreRobotics.Math.RadToDeg(pi/2); catch ME; k=k+1;disp('Math.DegToRad failed'); end
try out = CoreRobotics.Math.RK4Integrator(@(t,x,u,p)-x+u,0.01,0,0,1,[]); catch ME; k=k+1;disp('Math.RK4Integrator failed'); end
try out = CoreRobotics.Math.WrapAngleToPi(pi+pi/6); catch ME; k=k+1;disp('Math.WrapAngleToPi failed'); end

%% CoreRobotics.Frame
try out = CoreRobotics.Math.DegToRad(90); catch ME; k=k+1;disp('Math.DegToRad failed'); end
try out = CoreRobotics.Math.RadToDeg(pi/2); catch ME; k=k+1;disp('Math.DegToRad failed'); end
try out = CoreRobotics.Math.RK4Integrator(@(t,x,u,p)-x+u,0.01,0,0,1,[]); catch ME; k=k+1;disp('Math.RK4Integrator failed'); end
try out = CoreRobotics.Math.WrapAngleToPi(pi+pi/6); catch ME; k=k+1;disp('Math.WrapAngleToPi failed'); end

%%
if k > 0, fprintf('CoreRobotics test failed with %i error codes\n',k);
else fprintf('CoreRobotics test succeeded!\n'); end
rmpath ../