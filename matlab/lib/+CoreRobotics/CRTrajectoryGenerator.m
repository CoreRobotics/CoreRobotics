classdef CRTrajectoryGenerator < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = CRTrajectoryGenerator(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(291, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = solve(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(292, self, varargin{:});
    end
    function varargout = step(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(293, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(294, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
