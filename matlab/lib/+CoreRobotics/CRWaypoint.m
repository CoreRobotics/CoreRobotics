classdef CRWaypoint < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function varargout = time(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(270, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(271, self, varargin{1});
      end
    end
    function varargout = position(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(272, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(273, self, varargin{1});
      end
    end
    function varargout = velocity(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(274, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(275, self, varargin{1});
      end
    end
    function varargout = acceleration(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(276, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(277, self, varargin{1});
      end
    end
    function varargout = jerk(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(278, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(279, self, varargin{1});
      end
    end
    function self = CRWaypoint(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(280, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(281, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
