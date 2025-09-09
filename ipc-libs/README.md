### IPC Libraries, Includes, and Binaries for ARM–DSP Communication.

This directory contains prebuilt libraries, includes, and binaries required for IPC (Inter-Process Communication) between ARM and DSP.
During the **trik-distro** firmware build, these components are automatically downloaded and included into the TRIK Linux firmware.

If you want to build these artifacts manually, you need to download the **Texas Instruments Processor SDK**:
[Processor SDK for OMAPL138 Processors for Linux and TI-RTOS Support](https://www.ti.com/tool/PROCESSOR-SDK-OMAPL138) (PROCESSOR-SDK-RTOS-OMAPL138).

#### 1. Building RTOS Libraries
In the root directory of `processor_sdk_rtos_omapl138_6_03_00_106`, run:

```bash
export SDK_INSTALL_PATH=/path/to/texas-tools/
export TOOLS_INSTALL_PATH=/path/to/texas-tools/
source ./setupenv.sh
make ipc_bios
```

#### 2. Building IPC Libraries
First, export the TRIK SDK environment:
```bash
. /opt/trik-sdk/environment-setup-arm926ejse-oe-linux-gnueabi
```

Then, export the build environment for IPC libraries, script `trik-env.sh` in `scripts/` directory:
```bash
source trik-env.sh TI_DEPOT=/path/to/ti-tools TRIK_SDK=/opt/trik-sdk KERNEL_INSTALL=/path/to/trik-distro/tmp-glibc/work-shared/trikboard/kernel-build-artifacts
```

After that, in the root directory of `ipc_3_50_04_08`, run:
```
make -e -f ipc-linux.mak
make
make install
```

All the required libraries will appear in the `ipc-libs` directory.

#### 3. TI Examples
If you need to build the TI-provided examples, you must export additional environment variables. Refer to the comments inside the `trik-env.sh` file for details.