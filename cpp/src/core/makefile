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

# Project Name
PROJ = core

# Source files
SRCS  = CRClock.cpp

# Create an object file for each source file
OBJS = $(SRCS:.cpp=.o)

default: build

build: $(SRCS)
	$(CXX) $(CXXFLAGS) -c $^ && ld -r $(OBJS) -o  $(PROJ).o

$(PROJ): $(OBJS) 
	$(CXX) $(CXXFLAGS) $^ -o $@

%.o: %.cpp %.hpp
	$(CXX) $(CXXFLAGS) -c $^

debug: $(PROJ)
	valgrind $(VOPT) ./$(PROJ)

run: $(OBJS) 
	$(CXX) $(CXXFLAGS) $^ -o $(PROJ) && ./$(PROJ)

zip:
	zip $(PROJ).zip *.cpp *.hpp makefile

clean:
	rm *.o

