CXX = g++ -std=c++11
CXXFLAGS = -fPIC 
LDFLAGS = -shared
RM = rm -f
TARGET_L = crpi_lib.so

SRCS = crpi.cpp crcl_xml.cpp crpi_xml.cpp crpi_robot.cpp crpi_robot_xml.cpp crpi_abb.cpp crpi_allegro.cpp crpi_kuka_lwr.cpp crpi_robotiq.cpp crpi_schunk_sdh.cpp crpi_universal.cpp

DEPS = ../../Portable.h ../ulapi/src/ulapi.h crpi.h crpi_xml.h crpi_robot.h crpi_robot_xml.h crpi_abb.h crpi_allegro.h crpi_kuka_lwr.h crpi_robotiq.h crpi_schunk_sdh.h crpi_universal.h ../Math_Lib/NumericalMath.h ../Math_Lib/VectorMath.h ../Math_Lab/MatrixMath.h
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET_L)

$(TARGET_L): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $^

clean:
	$(RM) $(OBJS) $(TARGET_L)
