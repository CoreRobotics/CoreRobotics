classdef CRNoiseMixture < CoreRobotics.CRNoiseModel
  methods
    function self = CRNoiseMixture(varargin)
      self@CoreRobotics.CRNoiseModel(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(194, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = add(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(195, self, varargin{:});
    end
    function varargout = sample(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(196, self, varargin{:});
    end
    function varargout = probability(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(197, self, varargin{:});
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
