TRIK media sensors
==================
A set of useful camera algorithms for TRIK robots

Building
--------
1. Install required build tools:
  - [TI proccesor SDK for RTOS](https://www.ti.com/tool/PROCESSOR-SDK-OMAPL138) (tested on 06.03.00.106)
  - TI C6000 compiler (tested on 8.3.2) (included in RTOS package)
  - Also, TI library IMGlib is used (directory `dsp/lib`), but it's included in the repository to avoid some of the headache with building.
2. You need the IPC libraries, which are already available in the `ipc-libs` directory, or you can build them yourself by following the instructions in the `README.md` located in the corresponding directory.
3. In the directory `ipc_3_50_04_08/packages/ti/ipc/remoteproc`, you need to replace the file `rsc_table_omapl138.h`.
The modified version of this file with the required changes can be found in `ipc-libs/specific-trik-include`. The address and size values were modified according to the memory regions allocated in Linux for the DSP to ensure proper data exchange and that the resource table metadata is located at the expected addresses.
4. Run
  ```bash
  make TI_DEPOT=/path/to/texas-tools \
       TI_C6000=/path/to/texas-tools/ti-cgt-c6000_8.3.2 \
       KERNEL_INSTALL=/path/to/trik-kernel-installation \
       TRIK_SDK=/path/to/trik-sdk
  ```

Example if using our trik-distro (yocto) `KERNEL_INSTALL=/path/to/trik-distro/tmp-glibc/work-shared/trikboard/kernel-build-artifacts`

It'll build ARM and DSP part of the software both for debug and release in `./arm/bin` and `./dsp/bin` directories.

