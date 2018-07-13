classdef Frame < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = Frame(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(95, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = setFreeValue(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(96, self, varargin{:});
    end
    function varargout = getFreeValue(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(97, self, varargin{:});
    end
    function varargout = setRotation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(98, self, varargin{:});
    end
    function varargout = setTranslation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(99, self, varargin{:});
    end
    function varargout = setRotationAndTranslation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(100, self, varargin{:});
    end
    function varargout = getRotationAndTranslation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(101, self, varargin{:});
    end
    function varargout = getTransformToParent(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(102, self, varargin{:});
    end
    function varargout = getTransformToChild(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(103, self, varargin{:});
    end
    function varargout = transformToParent(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(104, self, varargin{:});
    end
    function varargout = transformToChild(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(105, self, varargin{:});
    end
    function varargout = isDriven(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(106, self, varargin{:});
    end
    function varargout = getTranslation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(107, self, varargin{:});
    end
    function varargout = getRotation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(108, self, varargin{:});
    end
    function varargout = getOrientation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(109, self, varargin{:});
    end
    function varargout = getPose(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(110, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(111, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
