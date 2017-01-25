% CoreRobotics.Manipulator provides access to serial chain robotic
% manipulator kinematics and methods for easily computing the forward
% kinematics and jacobians.
% 
% Usage:
% 
%   Use AddLink(<a href="matlab:help CoreRobotics.RigidBody">RigidBody</a> link) to add a rigid body frame to the
%   kinematic chain.  Use other methods to access the forward kinematics
%   and Jacobian of the manipulator, or set the driven variables.
% 
% Manipulator Properties:
%
%   name              - character name of the Manipulator
%   list_links        - a list of the rigid body links comprising the Manipulator
%   list_parents      - a list of the parents to the links
%   list_driven_links - a list of the driven rigid body links
%   size_links        - number of links in the list_links
%   size_driven_links - number of links in the list_driven_links
%   tool_frame        - frame of the tool pose
%
% Manipulator Methods:
%
%   AddLink              - adds a rigid body link to the list of links
%   SetDrivenVariables   - sets the driven variable values
%   GetDrivenVariables   - gets the driven variable values
%   GetForwardKinematics - returns the forward kinematics for the given driven variables
%   GetJacobian          - returns the numerical Jacobian for the Manipulator
%   
% Example:
%
%   T(1) = CoreRobotics.Frame2D('ang_a',0,'free_variables','ang_a');
%   T(2) = CoreRobotics.Frame2D('ang_a',-pi/2,'pos_x',0.5,'free_variables','ang_a');
%   T(3) = CoreRobotics.Frame2D('pos_x',0.8);
%   Robot = CoreRobotics.Manipulator;
%   for k = 1:length(T)
%       Link = CoreRobotics.RigidBody('frame',T(k));
%       Robot = Robot.AddLink(Link);
%   end
%
% References:
%
%   [1] J. Craig, "Introduction to Robotics: Mechanics and Control",
%       Ed. 3, Pearson,  2004.
%
%   [2] R. Murray, Z. Li, and S. Sastry, "A Mathematical Introduction to
%       Robot Manipulation", Ed. 1, CRC Press, 1993. <a href="http://www.cds.caltech.edu/~murray/mlswiki/index.php/Main_Page">Available online.</a>
% 
classdef Manipulator < CoreRobotics.InputHandler
    
    properties
        name; % Character name of the manipulator
    end
    
    properties (SetAccess = protected)
        list_links = []; % List of the links of class CoreRobotics.RigidBody.
        list_parents = []; % List of indices for the parents of the corresponding link in list_links.
        list_driven_links = []; % List of indices for the links that have driven variables.
        tool_frame = CoreRobotics.Frame('dimension',3); % Frame object representing the tool pose
    end
    
    properties (Dependent)
        size_links; % Number of links in list_links and list_parents.
        size_driven_links; % Number of driven (free) variables in the list_links.
    end
    
    methods
        function obj = AddLink(obj,link)
            % obj = obj.AddLink(link) adds a link to the manipulator object
            % where link is an object of class CoreRobotics.RigidBody.
            % Note that the Manipulator class only currently supports
            % serial (chain) manipulators.
            if isa(link,'CoreRobotics.RigidBody')
                parent = obj.size_links;
                obj.list_links = [obj.list_links; link];
                obj.list_parents = [obj.list_parents; parent];
                if link.frame.HasFreeVariables
                    obj.list_driven_links = [obj.list_driven_links; obj.size_links];
                end
            else
                error('CoreRobotics:Manipulator:IncompatibleType',...
                	'A link must be an object of class CoreRobotics.RigidBody')
            end
        end
        
        function obj = SetDrivenVariables(obj,u)
            % obj = obj.SetDrivenVariables(u) sets the vector u as the free
            % variable values in the list of driven frames.  u must
            % have a size of M x 1, where M = is the number of driven
            % links.
            M = obj.size_driven_links;
            if CheckSize(obj,u,[M 1])
                l = obj.list_driven_links;
                for k = 1:M
                    var = obj.list_links(l(k)).frame.free_variables;
                    obj.list_links(l(k)).frame.(var) = u(k);
                end
            end
        end

        function y = GetDrivenVariables(obj)
            % y = GetDrivenVariables(obj) returns a vector y of the free 
            % variable values in the driven rigid body frames.
            M = obj.size_driven_links;
            l = obj.list_driven_links;
            y = zeros(M,1);
            for k = 1:M
                y(k)=obj.list_links(l(k)).frame.GetFreeVariables;
            end
        end
        
        function [FK,obj] = GetForwardKinematics(obj,u)
            % [FK,obj] = obj.GetForwardKinematics(u) computes the
            % matrix of points along the forward kinematics FK for the
            % driven (free) variables in the vector drivenValues.  u must
            % have a size of M x 1, where M is the number of driven links.
            % The order in which the values in 'u' are applied is 
            % consistent with the order in which the RigidBody frames
            % are  added to the list using the AddLink() method.  If 'u'
            % is not specified, then the method uses the current values
            % specified for the free variables in the frames.
            if nargin == 2
                obj.SetDrivenVariables(u);
            end
            % Apply forward frame of points back to origin frame
            FK = zeros(3,obj.size_links);
            for k = 1:obj.size_links
                i = k;
                while i~=0
                    T = obj.list_links(i).frame;
                    FK(:,k) = T.TransformToFramePrev(FK(:,k));
                	i = obj.list_parents(i);
                end
            end
        end
        
        function J = GetJacobian(obj,allLinksFlag)
            % J = obj.GetJacobian(allLinksFlag) returns the numerical 
            % Jacobian for the kinematic frame encoded in the 
            % Manipulator object. The Jacobian J = df/dq is an N x M matrix
            % where f(q) is the forward frame manifold and q is the 
            % vector of driven (free) parameters.  When the optional 
            % allLinksFlag is set to 1 (default = 0), J is a N x M x L 
            % matrix of the Jacobian at each frame origin.
            if nargin < 2
                allLinksFlag = 0;
            end
            if CheckValue(obj,'allLinksFlag',allLinksFlag,'numeric',{0,1})
                dq = 1e-9; % dq
                N = 3;
                M = obj.size_driven_links;
                L = obj.size_links;
                if allLinksFlag
                    J = zeros(N,M,L);
                else
                    J = zeros(N,M);
                end
                u0 = obj.GetDrivenVariables;
                l = obj.list_driven_links;
                for k = 1:M
                    du = zeros(M,1);
                    du(k) = dq;
                    F2 = obj.GetForwardKinematics(u0+du);
                    F1 = obj.GetForwardKinematics(u0-du);
                    D = (F2-F1)/(2*dq);
                    if allLinksFlag
                        J(:,k,:) = D;
                    else
                        J(:,k) = D(:,end);
                    end
                end
            end
        end
        
        % setters and getters
        function obj = set.name(obj,value)
            if ischar(value)
                obj.name = value;
            else
                warning('CoreRobotics:Manipulator:BadManipulatorName',...
                	'robot_name expects a char array.')
            end
        end
        function value = get.size_links(obj)
            value = length(obj.list_links);
        end
        function value = get.size_driven_links(obj)
            value = length(obj.list_driven_links);
        end
        function tool = get.tool_frame(obj)
            i = obj.size_links;
            M = eye(4);
            while i~=0
                T = obj.list_links(i).frame;
                M = T.GetTransformToPrev*M;
                i = obj.list_parents(i);
            end
            r = M(1:end-1,1:end-1);
            t = M(1:end-1,end);
            obj.tool_frame.SetRotationAndTranslation(r,t);
            tool = obj.tool_frame;
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
