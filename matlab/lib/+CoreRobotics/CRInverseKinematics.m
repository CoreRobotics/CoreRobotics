classdef CRInverseKinematics < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = CRInverseKinematics(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(208, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setRobot(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(209, self, varargin{:});
    end
    function varargout = setToolIndex(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(210, self, varargin{:});
    end
    function varargout = setEulerMode(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(211, self, varargin{:});
    end
    function varargout = getEulerMode(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(212, self, varargin{:});
    end
    function varargout = setTolerance(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(213, self, varargin{:});
    end
    function varargout = setMaxIter(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(214, self, varargin{:});
    end
    function varargout = getMaxIter(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(215, self, varargin{:});
    end
    function varargout = setStepSize(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(216, self, varargin{:});
    end
    function varargout = getStepSize(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(217, self, varargin{:});
    end
    function varargout = setDampingFactor(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(218, self, varargin{:});
    end
    function varargout = getDampingFactor(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(219, self, varargin{:});
    end
    function varargout = setSingularThresh(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(220, self, varargin{:});
    end
    function varargout = getSingularThresh(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(221, self, varargin{:});
    end
    function varargout = getJacInv(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(222, self, varargin{:});
    end
    function varargout = solve(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(223, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(224, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
