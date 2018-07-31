classdef NoiseUniform < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = NoiseUniform(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(182, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = setParameters(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(183, self, varargin{:});
    end
    function varargout = sample(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(184, self, varargin{:});
    end
    function varargout = probability(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(185, self, varargin{:});
    end
    function varargout = m_parameters(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(186, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(187, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(188, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
