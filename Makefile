# TODO Better makefile than this laughable one.
#  - dependency & path generation

all : solver

clean :
	rm solver core -rf

solver:
	g++ -std=c++11 -Wall -Werror -o solver main.cpp graph.cpp node.cpp common.cpp
