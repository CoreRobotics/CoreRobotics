classdef CRRigidBody < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = CRRigidBody(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(129, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setFrame(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(130, self, varargin{:});
    end
    function varargout = m_frame(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(131, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(132, self, varargin{1});
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(133, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
