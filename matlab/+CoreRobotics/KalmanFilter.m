% KalmanFilter
% 
% Usage:
% 
% 
% Example:
% 
% 
% References:
% 
% 
classdef KalmanFilter < handle

    properties 
        motion_model; % Todo: doc
        sensor_model; % Todo: doc
        estimated_state; % Todo: doc
        estimated_cov; % Todo: doc
        gain; % Todo: doc
    end
    
    methods
        function obj = KalmanFilter(motionModel,sensorModel)
            % Todo: doc & input checking (mainly check state dimension)
            obj.motion_model = motionModel;
            obj.sensor_model = sensorModel;
            obj.estimated_state = motionModel.state;
            obj.estimated_cov = motionModel.covariance;
        end
        function [x,P] = Step(obj,z,u)
            % Todo: doc & input checking
            
            % Perform KF routine
            % 1. Prediction
            [x_pred,P_pred] = obj.motion_model.PredictMotion(u);
            obj.sensor_model.state = x_pred;
            zHat = obj.sensor_model.PredictMeasurement;
            
            % 2. Kalman Gain
            H = obj.sensor_model.mat_h;
            R = obj.sensor_model.sensor_noise.sigma;
            obj.gain = P_pred*H'/(H*P_pred*H'+R);
            
            % 3. Correction
            x = x_pred + obj.gain*(z-zHat);
            P = (eye(size(P_pred))-obj.gain*H)*P_pred;
            obj.estimated_state = x;
            obj.estimated_cov = P;
            
            %
            obj.motion_model.state = obj.estimated_state;
            obj.motion_model.covariance = obj.estimated_cov;
        end
        
        % setters and getters
        % Todo: set checking
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
