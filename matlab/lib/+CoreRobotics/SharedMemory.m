classdef SharedMemory < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = SharedMemory(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(283, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(284, self);
        self.swigPtr=[];
      end
    end
    function varargout = addSignal(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(285, self, varargin{:});
    end
    function varargout = removeSignal(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(286, self, varargin{:});
    end
    function varargout = set(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(287, self, varargin{:});
    end
    function varargout = get(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(288, self, varargin{:});
    end
  end
  methods(Static)
  end
end
