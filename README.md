TRIK media sensors
==================
A set of useful camera algorithms for TRIK robots

Building
--------
1. Install required build tools:
  - TI proccesor SDK for RTOS (tested on 06.03.00.106)
  - TI C6000 compiler (tested on 8.3.2)
  After installation configure `products.mak` with proper paths.
  Also, TI library IMGlib is used, but it's included in the repository to avoid some of the headache with building.
2. Run `make` 
  It'll build ARM and DSP part of the software both for debug and release in `./arm/bin` and `./dsp/bin` directories.


