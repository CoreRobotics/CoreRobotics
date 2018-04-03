classdef CRNoiseGaussian < CoreRobotics.CRNoiseModel
  methods
    function self = CRNoiseGaussian(varargin)
      self@CoreRobotics.CRNoiseModel(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(177, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setParameters(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(178, self, varargin{:});
    end
    function varargout = sample(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(179, self, varargin{:});
    end
    function varargout = probability(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(180, self, varargin{:});
    end
    function varargout = m_parameters(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(181, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(182, self, varargin{1});
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
