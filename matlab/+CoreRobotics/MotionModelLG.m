% MotionModelLG
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
classdef MotionModelLG < handle

    properties (Access = public)
        mat_a; % A matrix in: x(k+1) = A*x(k) + B*u(k) + G*w(k)
        mat_b; % B matrix in: x(k+1) = A*x(k) + B*u(k) + G*w(k)
        mat_g; % G matrix in: x(k+1) = A*x(k) + B*u(k) + G*w(k)
        motion_noise; % process noise model
        state; % state vector
        covariance; % state covariance matrix
    end
    
    methods
        function obj = MotionModelLG(A,B,G,Q,x,P)
            % TODO: doc
            % Check dimensions
            [ra,ca] = size(A);
            [rb,cb] = size(B);
            [rg,cg] = size(G);
            [rq,cq] = size(Q);
            if nargin < 5
                x = zeros(ra,1);
                P = eye(ra);
            elseif nargin < 6
                P = eye(ra);
            end
            [rx,cx] = size(x);
            [rp,cp] = size(P);
            if (ra~=rb) || (ra~=rg) || (cg~=cq) || (rq~=cq) || (ra~=rx) || (rp~=cp) || (rp~=ra)
                error('CoreRobotics:MotionModelLG:IncompatibleDimensions',...
                    'A, B, G, Q, x or P dimensions are incompatible. See <a href="matlab:help CoreRobotics.MotionModelLG">MotionModelLG</a>')
            else
                obj.mat_a = A;
                obj.mat_b = B;
                obj.mat_g = G;
                obj.state = x;
                obj.covariance = P;
                mu = zeros(rq,1);
                obj.motion_noise = CoreRobotics.NoiseModelGaussian(mu,Q);
            end
        end
        
        % Get Rid of this, it's contained in SimulateMotion
        function [x,P] = PredictMotion(obj,u)
            % TODO: doc (predicts the ideal state and covariance)
            [r,c] = size(u);
            [rb,cb] = size(obj.mat_b);
            if (r~=cb) || (~isnumeric(u))
                error('CoreRobotics:MotionModelLG:IncompatibleDimensions',...
                    'u must be a %i x 1 numeric vector',cb)
            else
                P = obj.covariance;
                Q = obj.motion_noise.sigma;
                
                x = obj.mat_a*obj.state + obj.mat_b*u;
                P = obj.mat_a*P*obj.mat_a' +...
                    obj.mat_g*Q*obj.mat_g';
            end
        end
        
        function [x,P] = SimulateMotion(obj,u,w)
            % TODO: doc (samples noise to perform the simulation!)
            if nargin < 3
                w = obj.motion_noise.Sample(1);
            else
                [~,cg] = size(obj.mat_g);
                [rw,cw] = size(w);
                if (~isnumeric(w)) || (rw~=cg) || (cw~=1)
                    error('CoreRobotics:MotionModelLG:IncompatibleNoise',...
                        'w must be a column vector with the same number of rows as transpose(mat_g)')
                end
            end
            [r,c] = size(u);
            [rb,cb] = size(obj.mat_b);
            if (r~=cb) || (~isnumeric(u))
                error('CoreRobotics:MotionModelLG:IncompatibleDimensions',...
                    'u must be a %i x 1 numeric vector',cb)
            else
                P = obj.covariance;
                Q = obj.motion_noise.sigma;
                
                x = obj.mat_a*obj.state + obj.mat_b*u + obj.mat_g*w;
                P = obj.mat_a*P*obj.mat_a' +...
                    obj.mat_g*Q*obj.mat_g';
            end
        end
        
        % setters and getters
        function set.mat_a(obj,value)
            [r,c] = size(value);
            if (r~=c) || (~isnumeric(value))
                error('CoreRobotics:MotionModelLG:IncompatibleDynamicsMatrix',...
                        'mat_a must be an N x N numeric')
            else
                obj.mat_a = value;
            end
        end
        function set.mat_b(obj,value)
            if (~isnumeric(value))
                error('CoreRobotics:MotionModelLG:IncompatibleInputMatrix',...
                        'mat_b must be an N x M numeric')
            else
                obj.mat_b = value;
            end
        end
        function set.mat_g(obj,value)
            if (~isnumeric(value))
                error('CoreRobotics:MotionModelLG:IncompatibleProcessMatrix',...
                        'mat_g must be an N x P numeric')
            else
                obj.mat_g = value;
            end
        end
        function set.motion_noise(obj,value)
            if (~isa(value,'CoreRobotics.NoiseModelGaussian'))
                error('CoreRobotics:MotionModelLG:IncompatibleNoiseModel',...
                        'motion_noise must be a <a href="matlab:help CoreRobotics.NoiseModelGaussian">NoiseModelGaussian</a> class')
            else
                obj.motion_noise = value;
            end
        end
        function set.state(obj,value)
            [r,c] = size(value);
            if (c~=1) || (~isnumeric(value))
                error('CoreRobotics:MotionModelLG:IncompatibleState',...
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
