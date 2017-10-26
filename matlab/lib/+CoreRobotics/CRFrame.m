classdef CRFrame < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = CRFrame(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(90, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setFreeValue(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(91, self, varargin{:});
    end
    function varargout = getFreeValue(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(92, self, varargin{:});
    end
    function varargout = setRotationAndTranslation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(93, self, varargin{:});
    end
    function varargout = getRotationAndTranslation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(94, self, varargin{:});
    end
    function varargout = getTransformToParent(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(95, self, varargin{:});
    end
    function varargout = getTransformToChild(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(96, self, varargin{:});
    end
    function varargout = transformToParent(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(97, self, varargin{:});
    end
    function varargout = transformToChild(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(98, self, varargin{:});
    end
    function varargout = isDriven(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(99, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(100, self, varargin{:});
    end
    function varargout = getOrientation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(101, self, varargin{:});
    end
    function varargout = getPose(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(102, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(103, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
