classdef CRFrameEuler < CoreRobotics.CRFrame
  methods
    function self = CRFrameEuler(varargin)
      self@CoreRobotics.CRFrame(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(103, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setFreeValue(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(104, self, varargin{:});
    end
    function varargout = getFreeValue(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(105, self, varargin{:});
    end
    function varargout = setFreeVariable(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(106, self, varargin{:});
    end
    function varargout = getFreeVariable(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(107, self, varargin{:});
    end
    function varargout = setMode(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(108, self, varargin{:});
    end
    function varargout = getMode(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(109, self, varargin{:});
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(110, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(111, self, varargin{:});
    end
    function varargout = setOrientation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(112, self, varargin{:});
    end
    function varargout = getOrientation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(113, self, varargin{:});
    end
    function varargout = setPositionAndOrientation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(114, self, varargin{:});
    end
    function varargout = getPositionAndOrientation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(115, self, varargin{:});
    end
    function varargout = isDriven(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(116, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(117, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
