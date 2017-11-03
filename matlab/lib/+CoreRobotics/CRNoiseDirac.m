classdef CRNoiseDirac < CoreRobotics.CRNoiseModel
  methods
    function self = CRNoiseDirac(varargin)
      self@CoreRobotics.CRNoiseModel(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(178, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setParameters(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(179, self, varargin{:});
    end
    function varargout = sample(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(180, self, varargin{:});
    end
    function varargout = probability(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(181, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(182, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
