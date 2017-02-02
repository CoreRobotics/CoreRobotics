% CoreRobotics.SensorModel is an abstract class that provides an interface
% to sensor (observation) representations.  Because it is abstract, 
% SensorModel cannot be instantiated as an object. Please see concrete 
% subclasses for implementations.
% 
% Usage:
% 
%   A general sensor (observation) model contains a system of equations that
%   map a sample-varying state x(k) to an output vector z(k), 
%   u(k) and noise v(k) for time t(k) of the form:
%
%       z(k) = h(x(k),u(k),v(k),m(k)).
%
%   Describing sensor models in this form allows for prediction of how
%   the underlying state changes affect the sensor output. Like the MotionModel
%   it is a powerful representation for estimation and control problems.  
%   The abstract SensorModel class sets up several methods that must be 
%   implemented in any subclass derived from the SensorModel class.
%
% MotionModel Properties:
%
%   state - the current state of the dynamic system
%
% SensorModel Methods:
%
%   Sensor - must implement a discrete-time model of the sensor behavior
%   Predict - predicts the sensor output for the current state vector
%
% See also <a href="matlab:help CoreRobotics.MotionModel">MotionModel</a>
%
% References:
%
%   [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic
%       Systems", Ed. 2, CRC Press, 2012.
% 
classdef SensorModel < CoreRobotics.InputHandler
    
    properties
        state; % the current state of the dynamic system
        noise_model; % measurement (sensor) noise model of class NoiseModel
    end
    
    methods (Abstract)
        z = Predict(obj);
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
