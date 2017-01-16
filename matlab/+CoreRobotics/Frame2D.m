% CoreRobotics.Frame2D provides access to homogeneous
% (translation and rotation) transformations in 2D.  It is derived from
% the CoreRobotics.Frame superclass.
% 
% Usage:
%
%   CoreRobotics.Frame2D constructs and applies the standard 
%   rotation and translation convention
%       T = [Rot(ang_a), Trans(pos_x,pos_y);
%            0,          1];
%   Pnew = T*P, where P is a 3 x 1 vector (augmented 1) representing a 
%   point in 2D Euclidean space.  For efficient transformation of points, 
%   use the methods built into the Frame2D class.
%
%   The transformation T as defined above takes a point in frame i (next)
%   to a point in frame i-1 (prev).
%
% Example:
%
%   myTform = CoreRobotics.Frame2D('ang_a',pi/4,'pos_x',1);
%   T0to1 = myTform.GetTransformToNext;
%   P1 = randn(2,101);
%   P0 = myTform.TransformToFramePrev(P1);
%
% References:
% 
%   [1] J. Craig, "Introduction to Robotics: Mechanics and Control",
%       Ed. 3, Pearson,  2004.
%
classdef Frame2D < CoreRobotics.Frame
    
    properties (Access = public)
        pos_x = 0; % X position: a 1x1 double (default = 0).
        pos_y = 0; % Y position: a 1x1 double (default = 0).
        ang_a = 0; % Rotation angle in radians: a 1x1 double (default = 0).
    end
    
    methods
        function obj = Frame2D(varargin)
            % obj = Frame2D(Name,Value) constructs the
            % Frame2D object for the Name/Value pairs, where Name 
            % is the property to be set on construct.
            obj.dimension = 2;
            obj.free_variables_values_ = {'pos_x','pos_y','ang_a'};
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
            % trans properties from the 2D transformation parameters.
            obj.rot = [cos(obj.ang_a), -sin(obj.ang_a);
                        sin(obj.ang_a),  cos(obj.ang_a)];
            obj.trans = [obj.pos_x; obj.pos_y];
        end 
        function obj = set.pos_x(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.pos_x = value;
                obj = SetRotationAndTranslation(obj);
            else
                error('CoreRobotics:Frame2D:IncompatibleType',...
                    'Property pos_x expects a numeric of size 1 x 1')
            end
        end
        function obj = set.pos_y(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.pos_y = value;
                obj = SetRotationAndTranslation(obj);
            else
                error('CoreRobotics:Frame2D:IncompatibleType',...
                    'Property pos_y expects a numeric of size 1 x 1')
            end
        end
        function obj = set.ang_a(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[1 1])
                obj.ang_a = CoreRobotics.Math.WrapAngleToPi(value);
                obj = SetRotationAndTranslation(obj);
            else
                error('CoreRobotics:Frame2D:IncompatibleType',...
                    'Property ang_a expects a numeric of size 1 x 1')
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
