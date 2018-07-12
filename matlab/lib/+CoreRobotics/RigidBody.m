classdef RigidBody < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function self = RigidBody(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(137, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(138, self);
        self.swigPtr=[];
      end
    end
    function varargout = setFrame(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(139, self, varargin{:});
    end
    function varargout = setCenterOfMass(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(140, self, varargin{:});
    end
    function varargout = getCenterOfMass(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(141, self, varargin{:});
    end
    function varargout = setInertiaTensor(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(142, self, varargin{:});
    end
    function varargout = getInertiaTensor(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(143, self, varargin{:});
    end
    function varargout = setMass(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(144, self, varargin{:});
    end
    function varargout = getMass(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(145, self, varargin{:});
    end
    function varargout = m_frame(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(146, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(147, self, varargin{1});
      end
    end
  end
  methods(Static)
  end
end
