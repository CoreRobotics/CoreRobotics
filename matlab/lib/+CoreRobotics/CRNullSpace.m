classdef CRNullSpace < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = CRNullSpace(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(229, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setRobot(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(230, self, varargin{:});
    end
    function varargout = setToolIndex(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(231, self, varargin{:});
    end
    function varargout = setEulerMode(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(232, self, varargin{:});
    end
    function varargout = getEulerMode(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(233, self, varargin{:});
    end
    function varargout = setSingularThresh(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(234, self, varargin{:});
    end
    function varargout = getSingularThresh(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(235, self, varargin{:});
    end
    function varargout = setMinStepSize(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(236, self, varargin{:});
    end
    function varargout = getMinStepSize(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(237, self, varargin{:});
    end
    function varargout = setMaxIter(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(238, self, varargin{:});
    end
    function varargout = getMaxIter(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(239, self, varargin{:});
    end
    function varargout = setTrivialTolerance(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(240, self, varargin{:});
    end
    function varargout = getTrivialTolerance(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(241, self, varargin{:});
    end
    function varargout = solve(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(242, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(243, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
