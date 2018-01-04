classdef mixtureParam < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function varargout = models(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(193, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(194, self, varargin{1});
      end
    end
    function varargout = weights(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(195, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(196, self, varargin{1});
      end
    end
    function self = mixtureParam(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(197, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(198, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
