classdef CRMath < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = CRMath(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(88, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(89, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = deg2rad(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(77, varargin{:});
    end
    function varargout = rad2deg(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(78, varargin{:});
    end
    function varargout = svd(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(79, varargin{:});
    end
    function varargout = svdInverse(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(80, varargin{:});
    end
    function varargout = rotAboutX(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(81, varargin{:});
    end
    function varargout = rotAboutY(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(82, varargin{:});
    end
    function varargout = rotAboutZ(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(83, varargin{:});
    end
    function varargout = normL1(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(84, varargin{:});
    end
    function varargout = normL2(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(85, varargin{:});
    end
    function varargout = normLinf(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(86, varargin{:});
    end
    function varargout = wrapToPi(varargin)
     [varargout{1:nargout}] = CoreRoboticsMEX(87, varargin{:});
    end
  end
end
