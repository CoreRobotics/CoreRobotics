classdef Clock < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = Clock(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(71, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(72, self);
        self.swigPtr=[];
      end
    end
    function varargout = startTimer(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(73, self, varargin{:});
    end
    function varargout = getElapsedTime(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(74, self, varargin{:});
    end
    function varargout = sleep(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(75, self, varargin{:});
    end
  end
  methods(Static)
  end
end
