% CoreRobotics.RigidBody provides an object that keeps track of objects
% associated with a single rigid body, such as the kinematic
% transformation, inertia tensor, and mass, etc...
% 
% Usage:
% 
%   The RigidBody object at a minimum consists of a transform property,
%   where the transform (object of class Frame, Frame2D, 
%   Frame3D, or FrameDH) is specified on construction.  
%   Because the RigidBody dimension property is inherited from the 
%   dimension of the transform object, it must be defined first if multiple
%   properties are set on construct.
% 
% Example:
% 
%   myLink = CoreRobotics.RigidBody('transform',...
%            	CoreRobotics.FrameDH('dh_r',2,'dh_theta',pi/2));
% 
% References:
% 
%   [1] J. Craig, "Introduction to Robotics: Mechanics and Control",
%       Ed. 3, Pearson,  2004.
% 
classdef RigidBody < CoreRobotics.InputHandler
    
    properties
        frame; % Frame, Frame2D, Frame3D or FrameDH object describing rigid body kinematics, with a consistent dimension.
        inertia; % Inertia object with a consistent dimension.
        mass; % Mass of the rigid body: a 1x1 double.
        center_of_mass; % Location of the center of mass with respect to the frame used for the frame. Must be a dimensionx1 vector.
    end
    
    properties (Dependent)
        dimension; % Dimension inherited from the frame
    end
    
    methods
        function obj = RigidBody(varargin)
            % obj = RigidBody(Name,Value) constructs the
            % RigidBody object for the Name/Value pairs, where Name 
            % is the property to be set on construct.  Note that at a
            % minimum, a frame object must be defined to make the
            % RigidBody object useful.  Because the dimension property is
            % derived from the frame property, this must be defined
            % before the other properties when constructing the object.
            if CheckVarargin(obj,varargin)
                for k = 1:2:length(varargin)
                    field = varargin{k};
                    value = varargin{k+1};
                    if (k == 1) && ~strcmp(field,'frame')
                        error('CoreRobotics:RigidBody:BadConstruct',...
                            ['Property ''frame'' must be defined on ',...
                            'construct, and must be the first property if ',...
                            'multiple properties are defined on construct']);
                    else
                        obj.(field) = value;
                    end
                end
            end
        end
        function obj = set.dimension(obj,value)
            if CheckValue(obj,'dimension',value,'numeric',{2,3})
                obj.dimension = value;
            end
        end
        function obj = set.frame(obj,value)
            if (isa(value,'CoreRobotics.Frame')||...
                    isa(value,'CoreRobotics.Frame2D')||...
                    isa(value,'CoreRobotics.Frame3D')||...
                    isa(value,'CoreRobotics.FrameDH'))
                obj.frame = value;
            else
                error('CoreRobotics:RigidBody:IncompatibleType',...
                      ['Property ''frame'' expects an object of',...
                       'class CoreRobotics.Frame, ',...
                       'CoreRobotics.Frame2D, ',...
                       'CoreRobotics.Frame3D, or ',...
                       'CoreRobotics.FrameDH'])
            end
        end
        function value = get.dimension(obj)
            value = obj.frame.dimension;
        end
        function obj = set.inertia(obj,value)
            if (isa(value,'CoreRobotics.Inertia'))
                if value.dimension == obj.dimension
                    obj.inertia = value;
                else
                    error('CoreRobotics:RigidBody:BadDimension',...
                        'Property ''inertia'' expects an object with dimension = %i',...
                        obj.dimension)
                end
            else
                error('CoreRobotics:RigidBody:IncompatibleType',...
                      'Property ''inertia'' expects an object of class CoreRobotics.Inertia')
            end
        end
        function obj = set.mass(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.mass = value;
            else
                error('CoreRobotics:RigidBody:IncompatibleType',...
                    'Property ''mass'' expects a numeric of size 1 x 1')
            end
        end
        function obj = set.center_of_mass(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[obj.dimension 1])
                obj.center_of_mass = value;
            else
                error('CoreRobotics:RigidBody:IncompatibleType',...
                    'Property ''center_of_mass'' expects a numeric of size %i x 1',...
                    obj.dimension)
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
