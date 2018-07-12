classdef HardLimits < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = HardLimits(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(256, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = getIKSolver(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(257, self, varargin{:});
    end
    function varargout = getNullSpaceSolver(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(258, self, varargin{:});
    end
    function varargout = useNullSpace(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(259, self, varargin{:});
    end
    function varargout = nullSpaceStatus(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(260, self, varargin{:});
    end
    function varargout = setPoseElements(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(261, self, varargin{:});
    end
    function varargout = getPoseElements(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(262, self, varargin{:});
    end
    function varargout = setJointUpperLimit(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(263, self, varargin{:});
    end
    function varargout = setJointLowerLimit(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(264, self, varargin{:});
    end
    function varargout = getJointUpperLimit(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(265, self, varargin{:});
    end
    function varargout = getJointLowerLimit(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(266, self, varargin{:});
    end
    function varargout = setJointLimits(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(267, self, varargin{:});
    end
    function varargout = setJointUpperLimits(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(268, self, varargin{:});
    end
    function varargout = setJointLowerLimits(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(269, self, varargin{:});
    end
    function varargout = getJointUpperLimits(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(270, self, varargin{:});
    end
    function varargout = getJointLowerLimits(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(271, self, varargin{:});
    end
    function varargout = setQ0(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(272, self, varargin{:});
    end
    function varargout = getQ0(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(273, self, varargin{:});
    end
    function varargout = setToolPose(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(274, self, varargin{:});
    end
    function varargout = getToolPose(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(275, self, varargin{:});
    end
    function varargout = setJointMotion(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(276, self, varargin{:});
    end
    function varargout = getJointMotion(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(277, self, varargin{:});
    end
    function varargout = solve(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(278, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(279, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
