CXX=g++			
CXXFLAGS  = -Wall   	
CXXFLAGS += -g
CXXFLAGS += -pedantic
CXXFLAGS += -std=c++11
LDFLAGS=         	

# Valgrind options
VOPT = --tool=memcheck --leak-check=full --show-leak-kinds=all --track-origins=yes

# Phony files... for tags that shouldn't generate files.  Without phony, make will check if they exist
# and if they do for some reason, they would never run
.PHONY: default debug clean zip run

# set folder names for individual libraries
CORE = core
PHYSICS = physics
MODELS = models

# combiled libraries
LIBS  = $(CORE)/$(CORE).o
LIBS += $(PHYSICS)/$(PHYSICS).o
LIBS += $(MODELS)/$(MODELS).o

# target dependencies
MY_TARGETS = $(CORE) $(PHYSICS) $(MODELS)

# Project Name
PROJ = CoreRobotics

# Source files
SRCS  = crTest.cpp

OBJS = $(SRCS:.cpp=.o)

default: all

.PHONY: $(CORE)
$(CORE):
	@$(MAKE) -C $@;

.PHONY: $(PHYSICS)
$(PHYSICS):
	@$(MAKE) -C $@;

.PHONY: $(MODELS)
$(MODELS): 
	@$(MAKE) -C $@;

.PHONY: all 
all: lib $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) $(PROJ).o -o $(PROJ).out
	
.PHONY: lib
lib: $(MY_TARGETS) 
	ld -r $(LIBS) -o $(PROJ).o
	
debug: all
	valgrind $(VOPT) ./$(PROJ).out

run: all
	./$(PROJ).out

clean:
	rm $(CORE)/*.o $(PHYSICS)/*.o $(MODELS)/*.o $(PROJ).out $(PROJ).o $(OBJS)