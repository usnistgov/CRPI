CXX = g++
CXXFLAGS = -fPIC 
LDFLAGS = -shared
RM = rm -f
TARGET_L = lib_RegistrationKit.so

SRCS = CoordFrameReg.cpp
DEPS = ../CRPI/crpi.h ../Math/MatrixMath.h ../../Clustering/kMeans/kMeansCluster.h CoordFrameReg.cpp
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET_L)

$(TARGET_L): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $^

clean:
	-$(RM) $(OBJS) $(TARGET_L)
