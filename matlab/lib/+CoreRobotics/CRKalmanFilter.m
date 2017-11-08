classdef CRKalmanFilter < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = CRKalmanFilter(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(270, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = getA(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(271, self, varargin{:});
    end
    function varargout = setA(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(272, self, varargin{:});
    end
    function varargout = getB(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(273, self, varargin{:});
    end
    function varargout = setB(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(274, self, varargin{:});
    end
    function varargout = getC(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(275, self, varargin{:});
    end
    function varargout = setC(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(276, self, varargin{:});
    end
    function varargout = getQ(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(277, self, varargin{:});
    end
    function varargout = setQ(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(278, self, varargin{:});
    end
    function varargout = getR(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(279, self, varargin{:});
    end
    function varargout = setR(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(280, self, varargin{:});
    end
    function varargout = getState(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(281, self, varargin{:});
    end
    function varargout = getCovariance(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(282, self, varargin{:});
    end
    function varargout = getTolerance(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(283, self, varargin{:});
    end
    function varargout = setTolerance(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(284, self, varargin{:});
    end
    function varargout = getStepSize(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(285, self, varargin{:});
    end
    function varargout = setStepSize(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(286, self, varargin{:});
    end
    function varargout = getIntegrationMethod(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(287, self, varargin{:});
    end
    function varargout = setIntegrationMethod(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(288, self, varargin{:});
    end
    function varargout = step(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(289, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(290, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
