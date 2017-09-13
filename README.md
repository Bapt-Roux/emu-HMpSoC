# emu-HMpSoC
Configurable HMpSoC emulation platform targeting the evaluation of HW/SW partitioning and mapping algorithms.

## Quick Start
### Clone the repository and init the submodules:
```shell
git clone https://github.com/rouxb/emu-HMpSoC ~/emu-HMpSoC
cd ~/emu-HMpSoC; git submodule update --init --recursive
```

### Install the build dependencies
This is just some clue to help the user during the setup process.
#### General
Install C/C++ compiler, cmake and others development tools.

#### SystemC dependencies
[SystemC provider website](http://www.accellera.org/downloads/standards/systemc "Accellera downloads page")

#### QT5
The application graph generator GUI used QT5. You should install QT if you want to build it otherwise you could use one of the graph example provides

#### Arm toolchain and Xilinx petalinux
If you want to modify the execution framework you should install a arm cross-compilation tool.
If you want to change the used linux image or recompile the provided kernel modules you should install petalinux tool from xilinx.


### Setup the build environnement and patch the submodules
```shell
# Create an folder for the out-of-tree compilation
cd ~/emu-HMpSoC; mkdir all-build; cd all-build;

# Initialize the build system
cmake ../ -DSYSTEMC_PREFIX=/{customPath}/systemc-2.3.1 -DPETALINUX_PREFIX=/{customPath}/petalinux-v2016.3-final

# Patch libsystemctlm-soc
cd ~/emu-HMpSoC/all-build/soc-env/;
make patch_libsystemctlm-soc

# Patch qemu-xlnx
cd ~/emu-HMpSoC/all-build/qemu-env/;
make patch_qemu

# Only if you want to use the prebuild package
cd ~/emu-HMpSoC/all-build/qemu-env/;
make load_binaries

```

### Build soc-env (vp_mainFm) and qemu-env (vp_qemuLauncher)
```shell
cd ~/emu-HMpSoC/all-build/soc-env/; make vp_mainFm
cd ~/emu-HMpSoC/all-build/qemu-env/; make vp_qemuLauncher
```

### Run prebuild version of the execution framework
Open two terminals, one for qemu instance and another one for systemC
#### Qemu side
```shell
cd ~/emu-HMpSoC/all-build/qemu-env/;
./vp_qemuLauncher ../../config-soc/configXX.json
```
#### SystemC side
```shell
cd ~/emu-HMpSoC/all-build/soc-env/;
./vp_mainFm ../../config-soc/configSocXX.json
# Warning: between two run, remove the ipc socket file
rm /tmp/emu-hmpsoc-tmp/nocIpc
```

#### Within each emulation cluster
Once the emulation platform has start (end of linux boot).
You should connect to each cluster with ssh and load the kernel modules.
Loox inside the configuration file in json to determine the port number
```shell
ssh localhost -p {offset + clusterId} -l root
pwd: root
cd /run/media/mmcblk0; insmod ker_modules/drv_monAlloc.ko; insmod ker_modules/drv_noc.ko; insmod ker_modules/drv_genIp.ko
```
Then you should start the application stub (slave and master)

```shell
./mutant_exec appsGraph/XX.mgr # master side
./mutant_slave # slave side
```
