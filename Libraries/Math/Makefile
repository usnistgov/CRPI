CXX = g++
CXXFLAGS = -fPIC 
LDFLAGS = -shared
RM = rm -f
TARGET_L = math_lib.so

SRCS = Filters.cpp NumericalMath.cpp VectorMath.cpp 
DEPS = ../../Portable.h Filters.h NumericalMath.h VectorMath.h MatrixMath.h
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET_L)

$(TARGET_L): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $^

clean:
	-$(RM) $(OBJS) $(TARGET_L)
