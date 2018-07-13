classdef SensorLinear < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = SensorLinear(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(220, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = setObservation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(221, self, varargin{:});
    end
    function varargout = measurement(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(222, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(223, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
