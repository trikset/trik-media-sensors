#!/bin/sh
clang-format -i \
    ./arm/src/*.c \
    ./arm/include/trik/sensors/*.h \
    ./dsp/src/*.c ./dsp/src/*.cpp \
    ./dsp/include/trik/sensors/*.h ./dsp/include/trik/sensors/*.hpp \
    ./shared/include/trik/sensors/*.h
