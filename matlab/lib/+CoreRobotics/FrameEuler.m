classdef FrameEuler < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = FrameEuler(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(112, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = setFreeValue(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(113, self, varargin{:});
    end
    function varargout = getFreeValue(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(114, self, varargin{:});
    end
    function varargout = setFreeVariable(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(115, self, varargin{:});
    end
    function varargout = getFreeVariable(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(116, self, varargin{:});
    end
    function varargout = setMode(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(117, self, varargin{:});
    end
    function varargout = getMode(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(118, self, varargin{:});
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(119, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(120, self, varargin{:});
    end
    function varargout = setOrientation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(121, self, varargin{:});
    end
    function varargout = getOrientation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(122, self, varargin{:});
    end
    function varargout = setPositionAndOrientation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(123, self, varargin{:});
    end
    function varargout = getPositionAndOrientation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(124, self, varargin{:});
    end
    function varargout = setRotationAndTranslation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(125, self, varargin{:});
    end
    function varargout = isDriven(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(126, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(127, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
