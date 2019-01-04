classdef TrajectoryGenerator < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = TrajectoryGenerator(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(301, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = solve(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(302, self, varargin{:});
    end
    function varargout = step(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(303, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(304, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
