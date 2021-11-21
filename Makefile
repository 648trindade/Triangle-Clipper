CXX=g++

all: main.cpp geometry.hpp
	${CXX} main.cpp -O2 -std=c++17 -o triangle_clipper -lm
run: all
	./triangle_clipper 1 -v
