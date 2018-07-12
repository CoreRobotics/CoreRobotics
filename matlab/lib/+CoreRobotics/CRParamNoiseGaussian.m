classdef CRParamNoiseGaussian < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function varargout = cov(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(176, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(177, self, varargin{1});
      end
    end
    function varargout = covInv(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(178, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(179, self, varargin{1});
      end
    end
    function varargout = mean(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(180, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(181, self, varargin{1});
      end
    end
    function self = CRParamNoiseGaussian(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(182, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(183, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
