classdef NullSpace < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = NullSpace(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(244, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = setRobot(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(245, self, varargin{:});
    end
    function varargout = setToolIndex(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(246, self, varargin{:});
    end
    function varargout = setEulerMode(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(247, self, varargin{:});
    end
    function varargout = getEulerMode(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(248, self, varargin{:});
    end
    function varargout = setSingularThresh(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(249, self, varargin{:});
    end
    function varargout = getSingularThresh(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(250, self, varargin{:});
    end
    function varargout = setMinStepSize(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(251, self, varargin{:});
    end
    function varargout = getMinStepSize(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(252, self, varargin{:});
    end
    function varargout = setMaxIter(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(253, self, varargin{:});
    end
    function varargout = getMaxIter(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(254, self, varargin{:});
    end
    function varargout = setTrivialTolerance(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(255, self, varargin{:});
    end
    function varargout = getTrivialTolerance(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(256, self, varargin{:});
    end
    function varargout = solve(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(257, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(258, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
