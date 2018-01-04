classdef CRNoiseDirac < CoreRobotics.CRNoiseModel
  methods
    function self = CRNoiseDirac(varargin)
      self@CoreRobotics.CRNoiseModel(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(181, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setParameters(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(182, self, varargin{:});
    end
    function varargout = sample(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(183, self, varargin{:});
    end
    function varargout = probability(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(184, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(185, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
