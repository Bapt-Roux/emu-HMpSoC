#!/bin/bash
###
# This script update appsContainers with new elf files specify in args
########################################
set -e

#define rootFs name
CONTAINER=sdImage_pkg.bin

if [ $# -eq 2 ];
then
    if [ ! -f "$1/$CONTAINER" ]; then
        echo "create sdimage package"
        dd if=/dev/zero of=$CONTAINER bs=1M count=64
        mkfs.ext4 $1/$CONTAINER
    fi

    ### find list of ARM binary and kernel modules
    ELF_FILES=`find $1 -name "*.elf"`
    KERNEL_MODULES=`find $1 -name "*.ko"`
    GRAPH_FILES=`find $2/mutantGui/genAppsGraph -name "*.mgr"`
    echo "Binary to load in RamDisk are:"
    for file in $ELF_FILES; do
        echo $file
    done
    echo "Kernel modules to load in RamDisk are:"
    for file in $KERNEL_MODULES; do
        echo $file
    done
    echo "Mutant apps Graph load in RamDisk are"
    for file in $GRAPH_FILES; do
        echo $file
    done

    #Mount ramdisk
    mkdir -p $1\/tmpContainer
    sudo mount -o loop $1/$CONTAINER $1\/tmpContainer
    # Update Binary
    for file in $ELF_FILES; do
        sudo cp $file $1\/tmpContainer
    done
    # Update modules
    sudo mkdir -p $1\/tmpContainer/ker_modules
    for file in $KERNEL_MODULES; do
        sudo cp $file $1\/tmpContainer/ker_modules
    done
    # Update appsGraph
    sudo mkdir -p $1\/tmpContainer/appsGraph
    for file in $GRAPH_FILES; do
        sudo cp $file $1\/tmpContainer/appsGraph
    done
    sudo umount $1\/tmpContainer
else
    echo "Bad number of arguments: "
    echo "\t arg1 => binary path;"
fi
