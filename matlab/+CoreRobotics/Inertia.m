% CoreRobotics.Inertia provides an object for handling properties and
% operations related to inertias.
% 
% Usage:
% 
%   The Inertia object consists of a mass, a center of mass vector, and
%   an inertia tensor.  
% 
% RigidBody Properties:
%
%   tensor         - inertia tensor
%   mass           - scalar mass
%   center_of_mass - location of the center of mass for the defined tensor
%
% Example:
% 
%   inertia = CoreRobotics.Inertia('tensor',diag([3 0.1 0.1]));
% 
% References:
% 
%   [1] J. Craig, "Introduction to Robotics: Mechanics and Control",
%       Ed. 3, Pearson,  2004.
% 
classdef Inertia < CoreRobotics.InputHandler

    properties
        tensor = eye(3); % 3 x 3 matrix containing the symmetric inertia tensor
        mass = 1; % scalar mass of the object (user units)
        center_of_mass = zeros(3,1); % 3 x 1 vector containing the center of mass for which the tensor is defined
    end
    
    methods
        function obj = Inertia(varargin)
            % obj = Inertia(Name,Value) constructs the
            % Inertia object for the Name/Value pairs, where Name 
            % is the property to be set on construct.
            if CheckVarargin(obj,varargin)
                for k = 1:2:length(varargin)
                    field = varargin{k};
                    value = varargin{k+1};
                    obj.(field) = value;
                end
            end
        end
        
        % Setters and getters
        function obj = set.tensor(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[3 3])
                if issymmetric(value)
                    obj.tensor = value;
                else
                    error('CoreRobotics:Inertia:BadTensor',...
                        'tensor must be symmetric')
                end
            end
        end
        function obj = set.mass(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.mass = value;
            end
        end
        function obj = set.center_of_mass(obj,value)
            if isnumeric(value) && obj.CheckSize(value,[3 1])
                obj.center_of_mass = value;
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
