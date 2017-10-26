classdef CRSensorLinear < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = CRSensorLinear(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(204, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setObservation(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(205, self, varargin{:});
    end
    function varargout = measurement(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(206, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(207, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end