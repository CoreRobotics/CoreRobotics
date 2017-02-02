% SensorModelLG
% 
% Usage:
% 
% 
% Example:
% 
% 
% References:
% 
%   [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic
%       Systems", Ed. 2, CRC Press, 2012.
% 
classdef SensorModelLG < handle

    properties (Access = public)
        mat_h; % H matrix in: z(k) = H*x(k) + v(k)
        sensor_noise; % observation noise model
        state; % state vector
    end
    
    methods
        function obj = SensorModelLG(H,R,x)
            % TODO: doc
            % Check dimensions
            [rh,ch] = size(H);
            [rr,cr] = size(R);
            if nargin < 3
                x = zeros(ch,1);
            end
            [rx,cx] = size(x);
            if (rr~=rh) || (rr~=cr) || (rx~=ch)
                error('CoreRobotics:SensorModelLG:IncompatibleDimensions',...
                    'H, R or x dimensions are incompatible. See <a href="matlab:help CoreRobotics.SensorModelLG">SensorModelLG</a>')
            else
                obj.mat_h = H;
                obj.state = x;
                mu = zeros(rr,1);
                obj.sensor_noise = CoreRobotics.NoiseModelGaussian(mu,R);
            end
        end
        
        % Get Rid of this, it's contained in SimulateMotion
        function z = PredictMeasurement(obj)
            % TODO: doc
            z = obj.mat_h*obj.state;
        end
        
        function z = SimulateMeasurement(obj,v)
            % TODO: doc
            if nargin < 2
                v = obj.sensor_noise.Sample(1);
            else
                [rh,~] = size(obj.mat_h);
                [rv,cv] = size(v);
                if (~isnumeric(v)) || ((rv~=rh) || (cv~=1)) && (v~=0)
                    error('CoreRobotics:SensorModelLG:IncompatibleNoise',...
                        'v must be a column vector with the same number of rows as mat_h')
                end
            end
            z = obj.mat_h*obj.state + v;
        end
        
        % setters and getters
        function set.mat_h(obj,value)
            if (~isnumeric(value))
                error('CoreRobotics:SensorModelLG:IncompatibleDynamicsMatrix',...
                        'mat_h must be a T x N numeric')
            else
                obj.mat_h = value;
            end
        end
        function set.sensor_noise(obj,value)
            if (~isa(value,'CoreRobotics.NoiseModelGaussian'))
                error('CoreRobotics:SensorModelLG:IncompatibleNoiseModel',...
                        'sensor_noise must be a <a href="matlab:help CoreRobotics.NoiseModelGaussian">NoiseModelGaussian</a> class')
            else
                obj.sensor_noise = value;
            end
        end
        function set.state(obj,value)
            [r,c] = size(value);
            if (c~=1) || (~isnumeric(value))
                error('CoreRobotics:SensorModelLG:IncompatibleState',...
                        'state must be an N x 1 numeric')
            else
                obj.state = value;
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
