% CoreRobotics.Math collects mathematical methods into a single static 
% method class to be used by the other classes.
% 
% References:
% 
%    [1] E. Kreyszig, "Advanced Engineering Mathematics", Ed. 10, John 
%        Wiley & Sons, 2011.
% 
classdef Math < handle
    
    methods (Static)
            
        function newAngle = WrapAngleToPi(angle)
            % newAngle = WrapAngleToPi(angle) wraps the supplied angle 
            % to +/-pi.  Angles are in radians
            newAngle = mod(angle/(2*pi),1)*2*pi;
            newAngle(newAngle>pi) = newAngle(newAngle>pi)-2*pi;
        end
        
        function angleInRads = DegToRad(angleInDegrees)
            % angleInRads = DegToRad(angleInDegrees) converts angles from
            % degrees to radians.
            angleInRads = (pi/180).*angleInDegrees;
        end
        
        function angleInDegrees = RadToDeg(angleInRads)
            % angleInRads = DegToRad(angleInDegrees) converts angles from
            % radians to degrees.
            angleInDegrees = (180/pi).*angleInRads;
        end
        
        function x_t1 = RK4Integrator(fun_handle,dt,t0,x_t0,u_t0,params)
            % x_t1 = RK4Integrator(fun_handle,dt,t0,x_t0,u_t0,params)
            % performs a single step of Runge-Kutta 4 integration on the
            % continuous-time dynamic model contained in fun_handle.  To 
            % work properly, fun_handle must be defined as a set of first 
            % order differential equations according to:
            %
            % dx/dt = fun_handle(t,x,u,params)
            %
            % where t is the time [s], x is the N x M state vector, u is an
            % L x M input vector, and params is a user-defined parameter
            % variable.  fun_handle() must return the N x M derivative of 
            % the state vector (dx/dt).  fun_handle can be an explicitly
            % defined function or an anonymous function (see example).
            %
            % Example:
            %
            %   fun = @(t,x,u,p) (-x + u);
            %   x0 = 0;
            %   u0 = 0.5;
            %   x1 = CoreRobotics.Math.RK4Integrator(fun,0.01,0,x0,u0,[])
            if nargin < 6
                error('CoreRobotics:Math:NotEnoughInputs',...
                    'Not enough input arguments.')
            else
                f1 = fun_handle(t0,x_t0,u_t0,params);
                f2 = fun_handle(t0+dt/2,x_t0+(dt/2)*f1,u_t0,params);
                f3 = fun_handle(t0+dt/2,x_t0+(dt/2)*f2,u_t0,params);
                f4 = fun_handle(t0+dt,x_t0+dt*f3,u_t0,params);
                x_t1 = x_t0 + (dt/6)*(f1+2*f2+2*f3+f4);
            end
        end
    
        
    
    end
    
end

% *************************************************************************
% Copyright (c) 2017, CoreRobotics Library
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%     * Neither the name of "CoreRobotics Library" nor the
%       names of its contributors may be used to endorse or promote products
%       derived from this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS TO THE COREROBOTICS LIBRARY 
% BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
% GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
% HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
% OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
% *************************************************************************
