classdef CRWaypoint < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function varargout = time(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(289, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(290, self, varargin{1});
      end
    end
    function varargout = position(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(291, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(292, self, varargin{1});
      end
    end
    function varargout = velocity(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(293, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(294, self, varargin{1});
      end
    end
    function varargout = acceleration(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(295, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(296, self, varargin{1});
      end
    end
    function varargout = jerk(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(297, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(298, self, varargin{1});
      end
    end
    function self = CRWaypoint(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(299, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(300, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
