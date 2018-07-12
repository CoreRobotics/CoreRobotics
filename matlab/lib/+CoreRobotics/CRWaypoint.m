classdef CRWaypoint < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function varargout = time(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(286, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(287, self, varargin{1});
      end
    end
    function varargout = position(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(288, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(289, self, varargin{1});
      end
    end
    function varargout = velocity(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(290, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(291, self, varargin{1});
      end
    end
    function varargout = acceleration(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(292, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(293, self, varargin{1});
      end
    end
    function varargout = jerk(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = CoreRoboticsMEX(294, self);
      else
        nargoutchk(0, 0)
        CoreRoboticsMEX(295, self, varargin{1});
      end
    end
    function self = CRWaypoint(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(296, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(297, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
