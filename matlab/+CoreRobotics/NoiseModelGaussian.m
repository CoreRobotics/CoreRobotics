% NoiseModelGaussian
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
classdef NoiseModelGaussian < handle

    properties
        mu; % mean vector of the Gaussian normal noise model
        sigma; % covariance matrix of the Gaussian normal noise model
    end
    
    methods
        function obj = NoiseModelGaussian(mu,sigma)
            % TODO: doc
            [r1,c1] = size(mu);
            [r2,c2] = size(sigma);
            if (r1~=r2) || (r2~=c2) || (c1~=1)
                error('CoreRobotics:NoiseModelGaussian:IncompatibleDimensions',...
                    'mu and sigma dimensions are incompatible. See <a href="matlab:help CoreRobotics.NoiseModelGaussian">NoiseModelGaussian</a>')
            else
                obj.mu = mu;
                obj.sigma = sigma;
            end
        end
        
        function x = Sample(obj,N)
            % TODO: doc
            [n,~] = size(obj.mu);
            R = chol(obj.sigma);
            x = bsxfun(@plus,randn(N,n)*R,obj.mu')';
        end
        
        
        % setters and getters
        function set.mu(obj,value)
            [~,c] = size(value);
            if (c~=1) || (~isnumeric(value))
                error('CoreRobotics:NoiseModelGaussian:IncompatibleMu',...
                        'mu must be an N x 1 numeric')
            else
                obj.mu = value;
            end
        end
        
        function set.sigma(obj,value)
            [r,c] = size(value);
            if (r~=c) || (~isnumeric(value))
                error('CoreRobotics:NoiseModelGaussian:IncompatibleSigma',...
                        'sigma must be an N x N numeric')
            else
                if ~issymmetric(value)
                    error('CoreRobotics:NoiseModelGaussian:AsymmetricCovariance',...
                        'sigma must be a symmetric matrix.')
                else
                    [~,p] = chol(value);
                    if p~=0
                        error('CoreRobotics:NoiseModelGaussian:NonPosDefCovariance',...
                            'sigma must be a positive definite matrix.')
                    else
                        obj.sigma = value;
                    end
                end
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
