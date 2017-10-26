classdef CRManipulator < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = CRManipulator(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(135, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setConfiguration(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(136, self, varargin{:});
    end
    function varargout = getConfiguration(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(137, self, varargin{:});
    end
    function varargout = getForwardKinematics(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(138, self, varargin{:});
    end
    function varargout = getNumberOfLinks(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(139, self, varargin{:});
    end
    function varargout = getDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(140, self, varargin{:});
    end
    function varargout = getToolFrame(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(141, self, varargin{:});
    end
    function varargout = getLinkFrame(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(142, self, varargin{:});
    end
    function varargout = getToolPose(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(143, self, varargin{:});
    end
    function varargout = setModelType(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(144, self, varargin{:});
    end
    function varargout = jacobian(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(145, self, varargin{:});
    end
    function varargout = addLink(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(146, self, varargin{:});
    end
    function varargout = addTool(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(147, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(148, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
