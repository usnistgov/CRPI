CXX = g++
CXXFLAGS = -fPIC 
LDFLAGS = -shared
RM = rm -f
TARGET_L = sensorMoCap_lib.so

SRCS = OptiTrack.cpp Vicon.cpp
DEPS = ../../Math/MatrixMath.h ../../ThirdParty/Vicon/include/Client.h ../../ThirdParty/OptiTrack/include/NatNetTypes.h ../../ThirdParty/OptiTrack/include/NatNetClient.h MoCapTypes.h OptiTrack.h Vicon.h
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET_L)

$(TARGET_L): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $^

clean:
	-$(RM) $(OBJS) $(TARGET_L)
