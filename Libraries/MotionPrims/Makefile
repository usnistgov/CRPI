CXX = g++
CXXFLAGS = -fPIC 
LDFLAGS = -shared
RM = rm -f
TARGET_L = lib_motionprims.so

SRCS = AssemblyPrims.cpp
DEPS = ../CRPI/crpi.h ../CRPI/crpi_robot.h AssemblyPrims.h
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET_L)

$(TARGET_L): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $^

clean:
	-$(RM) $(OBJS) $(TARGET_L)
