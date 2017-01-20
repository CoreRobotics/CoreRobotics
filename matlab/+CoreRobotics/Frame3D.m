% CoreRobotics.Frame3D provides access to homogeneous
% (translation and rotation) transformations in 3D.  It is derived from
% the CoreRobotics.Frame superclass.
% 
% Usage:
%
%   CoreRobotics.Frame3D constructs and applies the standard 
%   rotation and translation convention
%       T = [Rot(ang_a,ang_b,ang_g), Trans(pos_x,pos_y,pos_z);
%            0,                      1];
%   Pnew = T*P, where P is a 4 x 1 vector (augmented 1) representing a 
%   point in 3D Euclidean space.  For efficient transformation of points, 
%   use the methods built into the Frame3D class.
%
%   The transformation T as defined above takes a point in frame i (next)
%   to a point in frame i-1 (prev).
%
%   Rotations are defined through combinations of Euler angles, with the 
%   convention specified by the convention property.  The default
%   convention is set to 'zxz'.  This represents the axis of rotation for
%   which the 'ang_a', 'ang_b', and 'ang_g' angle rotations are applied.
%   There are 12 standard allowable rotation methods.
%
% Frame3D Properties:
%
%   dimension                 - dimension of the frame transformation
%   free_variables            - free (driven) variables of the frame transformation
%   pos_x                     - x position
%   pos_y                     - y position
%   pos_z                     - z position
%   ang_a                     - alpha rotation angle (radians)
%   ang_b                     - beta rotation angle (radians)
%   ang_g                     - gamma rotation angle (radians)
%   convention                - Euler rotation convention (e.g.: 'xyz')
%
% Frame3D Methods:
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
%   myTform = CoreRobotics.Frame3D('ang_b',pi,'pos_z',5,'convention','xyx');
%   T1to0 = myTform.GetTransformToPrev;
%   P1 = randn(3,101);
%   P0 = myTform.TransformToFramePrev(P1);
%
% References:
% 
%   [1] J. Craig, "Introduction to Robotics: Mechanics and Control",
%       Ed. 3, Pearson,  2004.
%
classdef Frame3D < CoreRobotics.Frame

    
    properties (Access = public)
        convention = 'zxz'; % Euler angle convention (e.g.: 'zxz').
        pos_x = 0; % X position: a 1x1 double (default = 0).
        pos_y = 0; % Y position: a 1x1 double (default = 0).
        pos_z = 0; % Z position: a 1x1 double (default = 0).
        ang_a = 0; % Alpha angle (1st rotation) in radians: a 1x1 double (default = 0).
        ang_b = 0; % Beta angle (2nd rotation) in radians: a 1x1 double (default = 0).
        ang_g = 0; % Gamma angle (3rd rotation) in radians: a 1x1 double (default = 0).
    end
    
    
    methods
        function obj = Frame3D(varargin)
            % obj = Frame3D(Name,Value) constructs the
            % Frame3D object for the Name/Value pairs, where Name 
            % is the property to be set on construct.
            obj.dimension = 3;
            obj.free_variables_values = {'pos_x','pos_y','pos_z',...
                                         'ang_a','ang_b','ang_g'};
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
            % trans properties from the 3D transformation parameters.
            ang={'ang_a','ang_b','ang_g'};
            Rot = eye(3);
            for k = 1:3
                switch obj.convention(k)
                    case 'x', Rot = Rot*obj.rotx(obj.(ang{k}));
                    case 'y', Rot = Rot*obj.roty(obj.(ang{k}));
                    case 'z', Rot = Rot*obj.rotz(obj.(ang{k}));
                end
            end
            obj.rot = Rot;
            obj.trans = [obj.pos_x; obj.pos_y; obj.pos_z];
        end
        
        % Setters and getters
        function obj = set.convention(obj,value)
            if CheckValue(obj,'convention',value,'char',...
                    {'zxz','xyx','yzy','zyz','xzx','yxy',...
                    'xyz','yzx','zxy','xzy','zyx','yxz'})
                obj.convention = value;
                obj = SetRotationAndTranslation(obj);
            end
        end
        function obj = set.pos_x(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.pos_x = value;
                obj = SetRotationAndTranslation(obj);
            else
                error('CoreRobotics:Frame3D:IncompatibleType',...
                    'Property pos_x expects a numeric of size 1 x 1')
            end
        end
        function obj = set.pos_y(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.pos_y = value;
                obj = SetRotationAndTranslation(obj);
            else
                error('CoreRobotics:Frame3D:IncompatibleType',...
                    'Property pos_y expects a numeric of size 1 x 1')
            end
        end
        function obj = set.pos_z(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.pos_z = value;
                obj = SetRotationAndTranslation(obj);
            else
                error('CoreRobotics:Frame3D:IncompatibleType',...
                    'Property pos_z expects a numeric of size 1 x 1')
            end
        end
        function obj = set.ang_a(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.ang_a = CoreRobotics.Math.WrapAngleToPi(value);
                obj = SetRotationAndTranslation(obj);
            else
                error('CoreRobotics:Frame3D:IncompatibleType',...
                    'Property ang_a expects a numeric of size 1 x 1')
            end
        end
        function obj = set.ang_b(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.ang_b = CoreRobotics.Math.WrapAngleToPi(value);
                obj = SetRotationAndTranslation(obj);
            else
                error('CoreRobotics:Frame3D:IncompatibleType',...
                    'Property ang_b expects a numeric of size 1 x 1')
            end
        end
        function obj = set.ang_g(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.ang_g = CoreRobotics.Math.WrapAngleToPi(value);
                obj = SetRotationAndTranslation(obj);
            else
                error('CoreRobotics:Frame3D:IncompatibleType',...
                    'Property ang_g expects a numeric of size 1 x 1')
            end
        end
    end
    
    
    methods (Access = private)
        function Rot = rotx(obj,x)
            Rot = [ 1, 0,       0;
                    0, cos(x), -sin(x);
                    0, sin(x),  cos(x)];
        end
        function Rot = roty(obj,x)
            Rot = [ cos(x), 0,  sin(x);
                    0,      1,  0;
                   -sin(x), 0,  cos(x)];
        end
        function Rot = rotz(obj,x)
            Rot = [ cos(x), -sin(x), 0;
                    sin(x),  cos(x), 0;
                    0,       0,      1];
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
