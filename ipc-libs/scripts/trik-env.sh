#!/usr/bin/env bash

for arg in "$@"; do
    if [[ "$arg" == *=* ]]; then
        varname="${arg%%=*}"
        value="${arg#*=}"
        export "$varname"="$value"
    else
        echo "Warning: ignoring invalid argument '$arg', expected VAR=VALUE"
    fi
done

export PLATFORM=OMAPL138
export PREFIX_OPTION="--prefix=$TI_DEPOT/ipc_3_50_04_08/ipc-libs"
export TOOLCHAIN_LONGNAME=arm-oe-linux-gnueabi
export TOOLCHAIN_INSTALL_DIR="$TRIK_SDK/sysroots/x86_64-oesdk-linux/usr"
export TOOLCHAIN_PREFIX="$TOOLCHAIN_INSTALL_DIR/bin/$TOOLCHAIN_LONGNAME/$TOOLCHAIN_LONGNAME-"
export KERNEL_INSTALL_DIR="$KERNEL_INSTALL"
export SDKTARGETSYSROOT="$TRIK_SDK/sysroots/arm926ejse-oe-linux-gnueabi"
export CFLAGS="-marm -mcpu=arm926ej-s --sysroot=$SDKTARGETSYSROOT -fcommon"

# For build TI examples in ipc/examples directory
# export BIOS_INSTALL_DIR="$TI_DEPOT/bios_6_76_03_01"
# export IPC_INSTALL_DIR="$TI_DEPOT/ipc_3_50_04_08"
# export XDC_INSTALL_DIR="$TI_DEPOT/xdctools_3_55_02_22_core"

# just set it in ipc/products.mak file (troubles for vars with dots)
# ti.targets.elf.C674=$(TI_DEPOT)/ti-cgt-c6000_8.3.2

echo "Environment set up:"
echo "TI_DEPOT=$TI_DEPOT"
echo "TRIK_SDK=$TRIK_SDK"
echo "KERNEL_INSTALL=$KERNEL_INSTALL"

