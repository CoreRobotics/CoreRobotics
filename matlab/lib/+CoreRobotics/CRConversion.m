classdef CRConversion < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = CRConversion(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(79, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(80, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = deg2rad(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(76, varargin{:});
    end
    function varargout = rad2deg(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(77, varargin{:});
    end
    function varargout = wrapToPi(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(78, varargin{:});
    end
  end
end
