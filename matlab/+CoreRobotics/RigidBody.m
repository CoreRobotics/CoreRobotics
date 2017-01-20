% CoreRobotics.RigidBody provides an object that keeps track of objects
% associated with a single rigid body, such as the kinematic
% transformation, inertia tensor, and mass, etc...
% 
% Usage:
% 
%   The RigidBody object consists of a <a href="matlab:help CoreRobotics.Frame">Frame</a> class, and an <a href="matlab:help CoreRobotics.Inertia">Inertia</a> class,
%   both of which can be specified on construction.  The Frame dimension 
%   property must be 3 to construct a RigidBody object.
% 
% RigidBody Properties:
%
%   frame          - frame object for the rigid body transformation
%   inertia        - inertia object for the rigid body transformation
%
% Example:
% 
%   myLink = CoreRobotics.RigidBody('frame',CoreRobotics.FrameDH,...
%                                   'inertia',CoreRobotics.Inertia);
% 
% References:
% 
%   [1] J. Craig, "Introduction to Robotics: Mechanics and Control",
%       Ed. 3, Pearson,  2004.
% 
classdef RigidBody < CoreRobotics.InputHandler
    
    properties
        frame; % Frame, Frame2D, Frame3D or FrameDH object describing rigid body kinematics, with dimension property = 3.
        inertia; % Inertia object for rigid body dynamics.
    end
    
    methods
        function obj = RigidBody(varargin)
            % obj = RigidBody(Name,Value) constructs the
            % RigidBody object for the Name/Value pairs, where Name 
            % is the property to be set on construct.  Note that at a
            % minimum, a frame object should be defined to make the
            % RigidBody object useful.
            if CheckVarargin(obj,varargin)
                for k = 1:2:length(varargin)
                    field = varargin{k};
                    value = varargin{k+1};
                    obj.(field) = value;
                end
            end
        end
        
        % setters and getters
        function obj = set.frame(obj,value)
            if isa(value,'CoreRobotics.Frame')
                if value.dimension == 3
                    obj.frame = value;
                else
                    error('CoreRobotics:RigidBody:BadFrame',...
                    	'Property ''frame'' dimension property must be 3');
                end
            else
                error('CoreRobotics:RigidBody:IncompatibleType',...
                      ['Property ''frame'' expects an object of ',...
                       'class CoreRobotics.Frame'])
            end
        end
        function obj = set.inertia(obj,value)
            if obj.CheckClass(value,'CoreRobotics.Inertia')
                obj.inertia = value;
            end
        end
    end
end

% *************************************************************************
% Software License Agreement (BSD-3-Clause License)
% Copyright (c) 2017, CoreRobotics.
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are 
% met:
%
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%
%     * Neither the name of CoreRobotics nor the names of its contributors
%       may be used to endorse or promote products derived from this 
%       software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
% TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
% PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
% OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
% PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
% *************************************************************************
