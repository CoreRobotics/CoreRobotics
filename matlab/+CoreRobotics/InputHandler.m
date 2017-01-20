% CoreRobotics.InputHandler provides generic protected methods for handling
% function inputs.  CoreRobotics classes inherit this handle superclass for
% consistent input handling throughout the library.
classdef InputHandler < handle

    methods (Access = protected)
        
        function isGood = CheckVarargin(obj,args)
            % isGood = obj.CheckVarargin(args) checks the cell args of
            % Name/Value pairs to see if the Name is a property of the obj
            % and if args has an even number of terms.
            N = length(args);
            p = properties(obj);
            isGood = 0;
            if ~mod(N,2)
                isGood = 1;
                for k = 1:2:N
                    if sum(strcmp(args{k},p)) == 0
                        isGood = 0;
                        error('CoreRobotics:InputHandler:BadName',...
                            '''%s'' is not a valid property,',args{k})
                    end
                end
            else
                error('CoreRobotics:InputHandler:BadInput',...
                    'Input must be a valid property Name/Value pair')
            end
        end
        
        function isGood = CheckValue(obj,field,value,type,goodValues)
            % isGood = obj.CheckValue(field,value,type,goodValues) checks
            % 1) if the supplied value matches the indicated type (MATLAB 
            % types, e.g.: 'char', 'numeric' are currently supported).
            % 2) the supplied value against the cell goodValues that
            % contains the list of acceptible values.
            isGood = 0;
            if isa(value,type)
                switch type
                    case 'char'
                        if sum(strcmp(value,goodValues))>0
                            isGood = 1;
                        else
                            error('CoreRobotics:InputHandler:BadValue',...
                                '''%s'' is not a valid value for property %s',...
                                value,field)
                        end
                    case 'numeric'
                        if sum(cell2mat(goodValues)==value)>0
                            isGood = 1;
                        else
                            error('CoreRobotics:InputHandler:BadValue',...
                                '''%6.4f'' is not a valid value for property %s',...
                                value,field)
                        end
                end
            else
                error('CoreRobotics:InputHandler:IncompatibleType',...
                    'Property %s expects a value of type %s',field,type)
            end
        end
        
        function isGood = CheckSize(obj,value,sizeReqm)
            % isGood = obj.CheckSize(value,sizeReqm) checks the size of
            % value against the 1 x 2 vector sizeReqm indicating the
            % required size of value.
            isGood = 0;
            [m,n] = size(value);
            if (sizeReqm(1) == m)&&(sizeReqm(2) == n)
                isGood = 1;
            else
                error('CoreRobotics:InputHandler:BadSize',...
                    'Property expects a value with size %i x %i',...
                    sizeReqm(1),sizeReqm(2))
            end
        end
        
        
        
        function isGood = CheckClass(obj,value,classType)
            % isGood = obj.CheckClass(value,classType) checks the class of
            % value against the char classType.
            isGood = 0;
            if isa(value,classType)
                isGood = 1;
            else
                error('CoreRobotics:InputHandler:BadClass',...
                    'Property expects a class of type %s',classType)
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
