CXX = g++
CXXFLAGS = -fPIC 
LDFLAGS = -shared
RM = rm -f
TARGET_L = serial_lib.so

SRCS = serial.cpp
DEPS = ../../portable.h ../../types.h serial.h gccserial.h
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET_L)

$(TARGET_L): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $^

clean:
	-$(RM) $(OBJS) $(TARGET_L)
