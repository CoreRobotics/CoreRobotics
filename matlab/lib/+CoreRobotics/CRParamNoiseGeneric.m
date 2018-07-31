classdef CRParamNoiseGeneric < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function varargout = icdFunction(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(148, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(149, self, varargin{1});
      end
    end
    function varargout = probFunction(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(150, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(151, self, varargin{1});
      end
    end
    function self = CRParamNoiseGeneric(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(152, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(153, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
