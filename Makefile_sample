CXX = g++ -std=c++11
CXXFLAGS = -fPIC 
LDFLAGS = -g
LDLIBS = -L/../../Libraries/CRPI -lCRPI -I/usr/local/ulapi/include -L/usr/local/ulapi/lib -lulapi
RM = rm -f
TARGET = Source.out

SRCS = Source.cpp
DEPS = ../../Portable.h 
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LDLIBS)

clean:
	$(RM) $(OBJS) $(TARGET)
