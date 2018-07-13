classdef Manipulator < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = Manipulator(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(151, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(152, self);
        self.SwigClear();
      end
    end
    function varargout = setConfiguration(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(154, self, varargin{:});
    end
    function varargout = getConfiguration(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(155, self, varargin{:});
    end
    function varargout = getForwardKinematics(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(156, self, varargin{:});
    end
    function varargout = getNumberOfLinks(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(157, self, varargin{:});
    end
    function varargout = getDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(158, self, varargin{:});
    end
    function varargout = getToolFrame(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(159, self, varargin{:});
    end
    function varargout = getLinkFrame(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(160, self, varargin{:});
    end
    function varargout = getToolPose(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(161, self, varargin{:});
    end
    function varargout = jacobian(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(162, self, varargin{:});
    end
    function varargout = hessian(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(163, self, varargin{:});
    end
    function varargout = addLink(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(164, self, varargin{:});
    end
    function varargout = addTool(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(165, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = create(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(153, varargin{:});
    end
  end
end
