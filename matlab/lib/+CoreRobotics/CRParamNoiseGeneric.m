classdef CRParamNoiseGeneric < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function varargout = icdFunction(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(149, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(150, self, varargin{1});
      end
    end
    function varargout = probFunction(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(151, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(152, self, varargin{1});
      end
    end
    function self = CRParamNoiseGeneric(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(153, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(154, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
