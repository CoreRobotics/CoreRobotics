% CoreRobotics.Frame provides access to homogeneous
% (translation and rotation) transformations.
% 
% Usage:
% 
%   CoreRobotics.Frame constructs and applies the standard 
%   rotation and translation convention
%       T = [Rot, Trans;
%            0,   1];
%   Pnew = T*P, where P is a (N+1) x 1 vector (augmented 1) representing a 
%   point in N-D Euclidean space.  For efficient transformation of points, 
%   use the methods built into the Frame class.
%
%   The transformation T as defined above takes a point in frame i (next)
%   to a point in frame i-1 (prev).
% 
% Example:
%
%   myTform = CoreRobotics.Frame('dimension',3);
%   T1to0 = myTform.GetTransformToPrev;
%   P1 = randn(3,101);
%   P0 = myTform.TransformToFramePrev(P1); 
% 
% References:
% 
%   [1] J. Craig, "Introduction to Robotics: Mechanics and Control",
%       Ed. 3, Pearson,  2004.
% 
classdef Frame < CoreRobotics.InputHandler

    properties (Access = public)
        dimension = 3; % Dimension of the transform: 2 or 3 (default).
        free_variables = []; % Free variable of the transformation. Only 1 free variable allowed per transform at this time (default is empty).
    end
    
    properties (Access = protected)
        rot = eye(3); % (Protected) Rotation matrix data.
        trans = zeros(3,1); % (Protected) Translation matrix data.
        free_variables_values_ = {}; % (Protected) List of properties that can be set as free variables.
    end
    
    
    methods
        function obj = Frame(varargin)
            % obj = Frame(Name,Value) constructs the
            % Frame object for the Name/Value pairs, where Name 
            % is the property to be set on construct.
            if CheckVarargin(obj,varargin)
                for k = 1:2:length(varargin)
                    field = varargin{k};
                    value = varargin{k+1};
                    obj.(field) = value;
                end
            end
        end
        
        function obj = Set(obj,varargin)
            % obj = obj.Set(Name,Value) sets the Name/Value pairs for the 
            % Frame object.
            if CheckVarargin(obj,varargin)
               for k = 1:2:length(varargin)
                    field = varargin{k};
                    value = varargin{k+1};
                    obj.(field) = value;
                end
            end
        end
        
        function rot = GetRotation(obj)
            % rot = obj.GetRotation() returns the rotation matrix.
            rot = obj.rot;
        end
        
        function value = GetTranslation(obj)
            % rot = obj.GetTranslation() returns the translation matrix.
            value = obj.trans;
        end
        
        function T = GetTransformToNext(obj)
            % T = obj.GetTransformToNext() returns the homogeneous 
            % tranformation matrix T that takes points in frame i-1 to 
            % points in Frame i, e.g., pointIn1 = T*pointIn0.
            T = [obj.rot', -obj.rot'*obj.trans;
                 zeros(1,obj.dimension), 1];
        end
        
        function T = GetTransformToPrev(obj)
            % T = obj.GetTransformToPrev() returns the homogeneous 
            % tranformation matrix T that takes points in Frame i to points
            % in frame i-1, e.g., pointIn0 = T*pointIn1.
            T = [obj.rot, obj.trans;
                 zeros(1,obj.dimension), 1];
        end
        
        function Points = TransformToFrameNext(obj,PointsToTransform)
            % Points = obj.TransformToFrameNext(P) returns an array of 
            % Points in frame i, where P are a set of points in frame i-1.
            [N,~] = size(PointsToTransform);
            if N~=obj.dimension
                error('CoreRobotics:Frame:IncompatibleDimension',...
                 'Input points must be %i x N for a %iD Frame',...
                 obj.dimension,obj.dimension)
            else
                Points = bsxfun(@plus,obj.rot'*PointsToTransform,...
                         -obj.rot'*obj.trans);
                
            end
            
        end
        
        function Points = TransformToFramePrev(obj,PointsToTransform)
            % Points = obj.TransformToFramePrev(P) returns an array of 
            % Points in frame i-1, where P are a set of points in frame i.
            [N,~] = size(PointsToTransform);
            if N~=obj.dimension
                error('CoreRobotics:Frame:IncompatibleDimension',...
                 'Input points must be %i x N for a %iD Frame',...
                 obj.dimension,obj.dimension)
            else
                Points = bsxfun(@plus,obj.rot*PointsToTransform,...
                         obj.trans);
            end
            
        end
        
        function TF = HasFreeVariables(obj)
            % TF = obj.HasFreeVariables() returns a true/false indicating
            % if the object has a free variable specified.
            if ~isempty(obj.free_variables)
                TF = true;
            else
                TF = false;
            end
        end
        function obj = SetFreeVariables(obj,value)
            % obj = obj.SetFreeVariables(value) sets the free variable to
            % the input value.
            if HasFreeVariables(obj)
                if isnumeric(value) && CheckSize(obj,value,[1 1])
                    obj.(obj.free_variables) = value;
                else
                    error('CoreRobotics:Frame:BadInput',...
                        'SetFreeVariables expects a numeric of size 1x1')
                end
            else
                warning('CoreRobotics:Frame:NoFreeVars',...
                 'There are no Free Variables defined for this object')
            end
        end
        function value = GetFreeVariables(obj)
            % obj = obj.GetFreeVariables() returns the value of the free 
            % variable. If there is no free variable defined, the function
            % returns an empty value.
            if HasFreeVariables(obj)
                value = obj.(obj.free_variables);
            else
                value = [];
            end
        end
        function obj = SetRotationAndTranslation(obj,rot,trans)
            % obj = obj.SetRotationAndTranslation(rot,trans) sets the rot 
            % and trans properties from the rot and trans values.
            if isnumeric(rot) && CheckSize(obj,rot,obj.dimension*[1 1])
                if isnumeric(trans) && CheckSize(obj,trans,[obj.dimension 1])
                    obj.rot = rot;
                    obj.trans = trans;
                end
            end
        end
        function obj = PlotFrame(obj,h_axes,scale)
            % obj = obj.PlotFrame(h_axes,scale) plots the Frame object in
            % the axes specified by the h_axes.  If h_axes is not
            % specified, the current axes is used.  The scale determines
            % the scale factor used to plot the frame vectors.  The plotted
            % frame has a representation of x (red), y (green), and z
            % (blue).
            if nargin == 1
                h_axes = gca;
                scale = 1;
            elseif nargin == 2
                scale = 1;
            end
            axes(h_axes)
            N = obj.dimension;
            coor = [zeros(N,1), scale*eye(N)];
            coor = obj.TransformToFramePrev(coor);
            X = [coor(:,1), coor(:,2)];
            Y = [coor(:,1), coor(:,3)];
            if N == 2
                plot(X(1,:),X(2,:),'-r')
                plot(Y(1,:),Y(2,:),'-g')
            elseif N == 3
                Z = [coor(:,1), coor(:,4)];
                plot3(X(1,:),X(2,:),X(3,:),'-r')
                plot3(Y(1,:),Y(2,:),Y(3,:),'-g')
                plot3(Z(1,:),Z(2,:),Z(3,:),'-b')
            end
        end
        function obj = set.dimension(obj,value)
            if CheckValue(obj,'dimension',value,'numeric',{2,3})
                obj.dimension = value;
            end
        end
        function obj = set.free_variables(obj,value)
            if CheckValue(obj,'free_variables',value,'char',...
                    obj.free_variables_values_)
                obj.free_variables = value;
            end
        end
        function obj = set.rot(obj,value)
            if isnumeric(value) && CheckSize(obj,value,obj.dimension*[1 1])
                obj.rot = value;
            else
                error('CoreRobotics:Frame:BadRotation',...
                    'rot expects a numeric of size %i x %i',...
                    obj.dimension,obj.dimension)
            end
        end
        function obj = set.trans(obj,value)
            if isnumeric(value) && CheckSize(obj,value,[obj.dimension 1])
                obj.trans = value;
            else
                error('CoreRobotics:Frame:BadTranslation',...
                    'trans expects a numeric of size %i x 1',...
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
