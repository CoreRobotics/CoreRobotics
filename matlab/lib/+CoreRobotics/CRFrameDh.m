classdef CRFrameDh < CoreRobotics.CRFrame
  methods
    function self = CRFrameDh(varargin)
      self@CoreRobotics.CRFrame(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(118, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setFreeValue(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(119, self, varargin{:});
    end
    function varargout = getFreeValue(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(120, self, varargin{:});
    end
    function varargout = setFreeVariable(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(121, self, varargin{:});
    end
    function varargout = getFreeVariable(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(122, self, varargin{:});
    end
    function varargout = setMode(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(123, self, varargin{:});
    end
    function varargout = getMode(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(124, self, varargin{:});
    end
    function varargout = setParameters(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(125, self, varargin{:});
    end
    function varargout = getParameters(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(126, self, varargin{:});
    end
    function varargout = isDriven(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(127, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(128, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
