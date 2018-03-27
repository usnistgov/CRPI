CXX = g++
CXXFLAGS = -fPIC 
LDFLAGS = -shared 
RM = rm -f
TARGET_L = sensor_HRI.so

SRCS = LeapMotion.cpp ManusVR.cpp 

DEPLEAP = ../../ThirdParty/LeapSDK/include/Leap.h ../../ThirdParty/OpenCV2/include/opencv2/core/core.hpp ../../ThirdParty/OpenCV2/include/opencv2/calib3d.hpp ../../ThirdParty/OpenCV2/include/opencv2/highgui/highgui.hpp ../../ThirdParty/OpenCV2/include/opencv2/imgproc/imgprog.hpp ../../ThirdParty/OpenCV2/include/opencv2/contrib/contrib.hpp 
DEPMANUS = ../../ThirdParty/Manus/Manus.h
DEPGEN = ../../ulapi/src/ulapi.h ../../../portable.h ../../Math/MatrixMath.h

DEPS =  $(DEPGEN) $(DEPMANUS) $(DEPLEAP) -I/$(DEPMYO) LeapMotion.h ManusVR.h 

OBJS = $(SRCS:.cpp=.o)

all: $(TARGET_L)

$(TARGET_L): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $^

clean:
	-$(RM) $(OBJS) $(TARGET_L)
