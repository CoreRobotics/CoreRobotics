% CoreRobotics.MotionModel is an abstract class that provides an interface
% to discrete motion (dynamics) representations.  Because it is abstract, 
% MotionModel cannot be instantiated as an object. Please see concrete 
% subclasses for implementations.
% 
% Usage:
% 
%   A general motion (dynamics) model contains a system of equations that
%   describe the change in a sample-varying state x(k), subject to input 
%   u(k) and noise w(k) for time t(k) of the form:
%
%       x(k+1) = f(x(k),u(k),w(k),m(k))
%
%   Describing dynamic systems in this form allows for prediction of how
%   the underlying state changes, and is a powerful representation for
%   estimation and control problems.  The abstract MotionModel class sets
%   up several methods that must be implemented in any subclass derived
%   from the MotionModel class.
%
% MotionModel Properties:
%
%   state       - the current state of the dynamic system
%   noise_model - process (motion) noise model of class NoiseModel
%
% MotionModel Methods:
%
%   Motion - must implement a discrete-time model of the system behavior
%   Predict - predicts the state change for the current state vector
%
% See also <a href="matlab:help CoreRobotics.SensorModel">SensorModel</a>
%
% References:
%
%   [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic
%       Systems", Ed. 2, CRC Press, 2012.
% 
classdef MotionModel < CoreRobotics.InputHandler
    
    properties
        state; % the current state of the dynamic system
        noise_model; % process (motion) noise model of class NoiseModel
    end
    
    methods (Abstract)
        x = Predict(obj,u);
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
