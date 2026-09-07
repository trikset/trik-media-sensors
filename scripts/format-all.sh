#!/bin/sh
clang-format -i \
    ./dsp/src/*.c ./dsp/src/*.cpp \
    ./dsp/include/trik/sensors/*.h ./dsp/include/trik/sensors/*.hpp \
    ./shared/include/trik/sensors/*.h
