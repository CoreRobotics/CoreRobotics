classdef CRParamNoiseGaussian < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function varargout = cov(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(168, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(169, self, varargin{1});
      end
    end
    function varargout = covInv(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(170, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(171, self, varargin{1});
      end
    end
    function varargout = mean(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(172, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(173, self, varargin{1});
      end
    end
    function self = CRParamNoiseGaussian(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(174, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(175, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
