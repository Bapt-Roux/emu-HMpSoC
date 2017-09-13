#!/bin/bash

# This script create/update/build petalinux/qemu feature needed
# by HMpSoC virtual plateform simulator.

# Standard Path and name
BSP_TRGT=Xilinx-ZC702-v2016.3-final.bsp

# DTS_FROM=qemu-dts/zynq-pl-remoteport.dtsi
# DTS_TO=$PRJ_ROOT/$PTL_DIR/cosimLinux-build/subsystems/linux/configs/device-tree

if [ 3 -eq $# ]; then
    PTX_PATH=$1
    PTX_ROOT=$2
    PTX_DIR=$3

    source $PTX_PATH/settings.sh
    echo "Check existance of dir: $PTX_ROOT/$PTX_DIR"
    if [ ! -d "$PTX_ROOT/$PTX_DIR" ]; then
        #Create petalinux project
        cd $PTX_ROOT
        petalinux-create --type project -s $PTX_PATH/bsp/$BSP_TRGT --name $PTX_DIR
    else
        echo "SKIP: Petalinux project already exist."
    fi

    if [ ! -f $DTS_TO ]; then
        cp $DTS_FROM $DTS_TO
    else
        echo "SKIP: Device tree libremote already exist"
    fi

else
    echo "ERROR: PETALINUX PATH badly define in args list"
fi

#build petalinux
cd $PTX_ROOT/$PTX_DIR
petalinux-build
