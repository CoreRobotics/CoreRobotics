% CoreRobotics.FrameDH provides access to homogeneous
% (translation and rotation) transformations using the common kinematic
% Denavit-Hartenberg (DH) convention.  It is derived from
% the CoreRobotics.Frame superclass.
% 
% Usage:
%
%   The DH matrix is two screw displacement coordinate transformations
%   intended to simplify 3D robot rigid body kinematics to 4 parameters 
%   per link.  For more information on developing DH parameters, see 
%   references [1-2].  The transformation is designed to take a point from 
%   frame i to frame i-1, e.g.: PointIn0 = T*PointIn1. CoreRobotics.FrameDH 
%   constructs and applies one of the following DH conventions (specified 
%   by 'method'):
%
%       1. Classic DH parameter configuration
%           This convention constructs a transformation according to
%           T = Trans(dh_d)*Rot(dh_theta)*Trans(dh_r)*Rot(dh_alpha), where the first
%           translation/rotation operation set is about z_{i-1} and the 
%           second operation set is about common normal x_{i}.
%
%       2. Modified DH parameter configuration
%           This convention constructs a transformation according to
%           T = Rot(dh_alpha)*Trans(dh_r)*Rot(dh_theta)*Trans(dh_d), where the first
%           translation/rotation operation set is about about common normal
%           x_{i-1} and the second operation set is about z_{i}.
%
% FrameDH Properties:
%
%   dimension                 - dimension of the frame transformation
%   free_variables            - free (driven) variables of the frame transformation
%   dh_r                      - DH parameter r (sometimes 'a' in texts)
%   dh_alpha                  - DH parameter alpha (radians)
%   dh_d                      - DH parameter d
%   dh_theta                  - DH parameter theta (radians)
%   method                    - specify the DH convention ('modified' or 'classic')
%
% FrameDH Methods:
%
%   Set                       - sets name/value parameter pairs
%   GetRotation               - returns the rotation matrix
%   GetTranslation            - returns the translation matrix
%   GetTransformToNext        - returns the transformation to the next frame
%   GetTransformToPrev        - returns the transformation to the previous frame
%   TransformToFrameNext      - transforms a set of points to the next frame
%   TransformToFramePrev      - transforms a set of points to the previous frame
%   HasFreeVariables          - returns if the frame has free variables defined
%   SetRotationAndTranslation - sets the rotation and translation matrices
%   SetFreeVariables          - sets the defined free variable
%   GetFreeVariables          - gets the value of the free variable
%   PlotFrame                 - plots the i-j-k vectors associated with the frame
%
% Example:
%
%   myTform = CoreRobotics.FrameDH('dh_r',0.2,'dh_theta',pi/8,'method','modified');
%   T1to0 = myTform.GetTransformToPrev;
%   P1 = randn(3,101);
%   P0 = myTform.TransformToFramePrev(P1);
%
% References:
% 
%   [1] J. Denavit and R. Hartenberg, "A kinematic notation for lower-pair 
%       mechanisms based on matrices". Trans ASME J. Appl. Mech. 23,
%       pp. 215-221, 1955.
%
%   [2] J. Craig, "Introduction to Robotics: Mechanics and Control",
%       Ed. 3, Pearson,  2004.
%
classdef FrameDH < CoreRobotics.Frame


    properties (Access = public)
        method = 'modified'; % DH parameter method ('classic' or 'modified').
        dh_r = 0; % Length of the common normal x to new z (sometimes 'a' in textbooks): a 1x1 double (default = 0).
        dh_alpha = 0; % Angle about common normal x to new z in radians: a 1x1 double (default = 0).
        dh_d = 0; % Length of z, i.e. the distance between common normals: a 1x1 double (default = 0).
        dh_theta = 0; % Angle about z to new x in radians: a 1x1 double (default = 0).
    end
    
    methods
        function obj = FrameDH(varargin)
            % obj = FrameDH(Name,Value) constructs the
            % FrameDH object for the Name/Value pairs, where Name 
            % is the property to be set on construct.
            obj.dimension = 3;
            obj.free_variables_values = {'dh_r','dh_alpha','dh_d','dh_theta'};
            if CheckVarargin(obj,varargin)
                for k = 1:2:length(varargin)
                    field = varargin{k};
                    value = varargin{k+1};
                    if ~strcmp(field,'dimension')
                        obj.(field) = value;
                    end
                end
            end
        end
        function obj = SetRotationAndTranslation(obj)
            % obj = obj.SetRotationAndTranslation() sets the rot and 
            % trans properties from the DH transformation parameters.
            if strcmp(obj.method,'classic')
                % Original convention reported by Denavit & Hartenberg [1].
                obj.rot = [cos(obj.dh_theta),...
                      -sin(obj.dh_theta)*cos(obj.dh_alpha),...
                       sin(obj.dh_theta)*sin(obj.dh_alpha);
                       sin(obj.dh_theta),...
                       cos(obj.dh_theta)*cos(obj.dh_alpha),...
                      -cos(obj.dh_theta)*sin(obj.dh_alpha);
                       0, sin(obj.dh_alpha), cos(obj.dh_alpha)];
                obj.trans = [obj.dh_r*cos(obj.dh_theta);
                         obj.dh_r*sin(obj.dh_theta);
                         obj.dh_d];
            elseif strcmp(obj.method,'modified')
                % This is the convention used by Craig [2].
                obj.rot = [cos(obj.dh_theta), -sin(obj.dh_theta), 0;
                       sin(obj.dh_theta)*cos(obj.dh_alpha),...
                       cos(obj.dh_theta)*cos(obj.dh_alpha),...
                      -sin(obj.dh_alpha);
                       sin(obj.dh_theta)*sin(obj.dh_alpha),...
                       cos(obj.dh_theta)*sin(obj.dh_alpha),...
                       cos(obj.dh_alpha)];
                obj.trans = [obj.dh_r;
                        -obj.dh_d*sin(obj.dh_alpha);
                         obj.dh_d*cos(obj.dh_alpha)];
            end
        end
        
        % Setters and getters
        function obj = set.method(obj,value)
            if CheckValue(obj,'method',value,'char',{'classic','modified'})
                obj.method = value;
                obj = SetRotationAndTranslation(obj);
            end
        end
        function obj = set.dh_r(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.dh_r = value;
                obj = SetRotationAndTranslation(obj);
            else
                error('CoreRobotics:FrameDH:IncompatibleType',...
                    'Property dh_r expects a numeric of size 1 x 1')
            end
        end
        function obj = set.dh_d(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.dh_d = value;
                obj = SetRotationAndTranslation(obj);
            else
                error('CoreRobotics:FrameDH:IncompatibleType',...
                    'Property dh_d expects a numeric of size 1 x 1')
            end
        end
        function obj = set.dh_alpha(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.dh_alpha = CoreRobotics.Math.WrapAngleToPi(value);
                obj = SetRotationAndTranslation(obj);
            else
                error('CoreRobotics:FrameDH:IncompatibleType',...
                    'Property dh_alpha expects a numeric of size 1 x 1')
            end
        end
        function obj = set.dh_theta(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.dh_theta = CoreRobotics.Math.WrapAngleToPi(value);
                obj = SetRotationAndTranslation(obj);
            else
                error('CoreRobotics:FrameDH:IncompatibleType',...
                    'Property dh_theta expects a numeric of size 1 x 1')
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
