classdef vectorVectorXd < SwigRef
  methods
    function this = swig_this(self)
      this = CoreRoboticsMEX(3, self);
    end
    function varargout = pop(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(46, self, varargin{:});
    end
    function varargout = brace(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(47, self, varargin{:});
    end
    function varargout = setbrace(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(48, self, varargin{:});
    end
    function varargout = append(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(49, self, varargin{:});
    end
    function varargout = empty(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(50, self, varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(51, self, varargin{:});
    end
    function varargout = swap(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(52, self, varargin{:});
    end
    function varargout = begin(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(53, self, varargin{:});
    end
    function varargout = end(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(54, self, varargin{:});
    end
    function varargout = rbegin(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(55, self, varargin{:});
    end
    function varargout = rend(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(56, self, varargin{:});
    end
    function varargout = clear(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(57, self, varargin{:});
    end
    function varargout = get_allocator(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(58, self, varargin{:});
    end
    function varargout = pop_back(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(59, self, varargin{:});
    end
    function varargout = erase(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(60, self, varargin{:});
    end
    function self = vectorVectorXd(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = CoreRoboticsMEX(61, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = push_back(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(62, self, varargin{:});
    end
    function varargout = front(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(63, self, varargin{:});
    end
    function varargout = back(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(64, self, varargin{:});
    end
    function varargout = assign(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(65, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(66, self, varargin{:});
    end
    function varargout = insert(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(67, self, varargin{:});
    end
    function varargout = reserve(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(68, self, varargin{:});
    end
    function varargout = capacity(self,varargin)
      [varargout{1:nargout}] = CoreRoboticsMEX(69, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        CoreRoboticsMEX(70, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
