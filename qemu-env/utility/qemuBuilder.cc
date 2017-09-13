#include "qemuBuilder.h"

/*---
 * CLASS
 * METHODS IMPLEMENTATION
 *----------------------------------------------------------------------------*/
using namespace qb;
using namespace std;

#define KEEP(x) x
#define TO_STR(x) #x
#define _STR(x) TO_STR(x)
#define TO_PATH(x) _STR(KEEP(x))

qemuBuilder::qemuBuilder(ifstream & stream, char stype)
{

  char nocIpcName[64];
  cereal::importConfig<sb::socCnf>(topLevel, stream, stype);

  /*---
   * Qemu clusters spawning
   * --------------------------------------------------*/
  //allocate space for clusters pid
  clusters = new pid_t[topLevel.clusters.sMap.size()];
  uint clusterId=0;
  for (const auto & it_cluster: topLevel.clusters.sMap)
    {
      // spawn cluster Process
      clusters[clusterId] = fork();
      if(0 == clusters[clusterId]){
        //change to petalinux project path

        //build calling string
        string qemuArgs = TO_PATH(EMU_CUR_BIN) + (string)"/qemu-patch/bin/qemu-system-aarch64"
          + " -L " + TO_PATH(EMU_CUR_BIN)+"/qemu-patch/etc/qemu "
          + " -M arm-generic-fdt-plnx -machine linux=on   -serial /dev/null -serial mon:stdio -display none"
          + " -gdb tcp::"+ to_string(topLevel.global.gdb_port_offset+clusterId)
          + " -kernel " + TO_PATH(EMU_CUR_BIN) +"/"+ TO_PATH(PETALINUX_NAME)+"/images/linux/zImage" //FIXME set in json
          + " -initrd " + TO_PATH(EMU_CUR_BIN) +"/"+ TO_PATH(PETALINUX_NAME)+"/images/linux/rootfs.cpio.gz" //FIXME set in json
          + " -dtb "+ TO_PATH(EMU_TOP_BIN)+ "/qemu-env/" + it_cluster.second.cpu.dtb_name
          + " -tftp /home/broux/emu-hmpsoc/all-build/qemu-env/cosimPtx-build/images/linux"
          + " -device loader,addr=0xf8000008,data=0xDF0D,data-len=4 -device loader,addr=0xf8000140,data=0x00500801,data-len=4"
          + " -device loader,addr=0xf800012c,data=0x1ed044d,data-len=4 -device loader,addr=0xf8000108,data=0x0001e008,data-len=4"
          // + " -drive if=none,format=qcow2,file=" + TO_PATH(EMU_CUR_BIN)+"/qemuSave.qcow2"
          // + " -loadvm emuHmpsoc"
          + " -sd "+ TO_PATH(EMU_CUR_BIN)+"/sdImageCl"+to_string(clusterId) + ".bin"
          + " -machine-path "+ topLevel.global.sk_bpath + "/cl@"+to_string(clusterId)
          + " -icount " + to_string(topLevel.global.sync_icount)
          + " -sync-quantum " + to_string(topLevel.global.sync_quantum)
          + " -net nic -net user,hostfwd=tcp:127.0.0.1:"+ to_string(topLevel.global.ssh_port_offset + clusterId)
          + "-10.0.2.15:22";

        //Source petlinux script and start Qemu
        string systemCall = (string)"mkdir -p " + topLevel.global.sk_bpath+"; " //create top socket dir
          + (string)"mkdir -p " + topLevel.global.sk_bpath+ +"/cl@" + to_string(clusterId) +"; " //create cluster socket dir
          + (string)"cp " + TO_PATH(EMU_TOP_BIN)+"/apps-env/sdImage_pkg.bin "+ TO_PATH(EMU_CUR_BIN)+"/sdImageCl" + to_string(clusterId) +".bin; " //duplicate appsContainers
          + (string)"cd " + TO_PATH(EMU_CUR_BIN)+"/"+TO_PATH(PETALINUX_NAME) +"; " // change to petalinux prj dir
          + qemuArgs; //start Qemu

        system(systemCall.c_str());
        exit(1);
      }else if(0> clusters[clusterId]) {
        cerr << "Fork failed for Cluster: "<< it_cluster.first << ", Id: " <<clusterId << endl;
        exit(1);
      }else{ /*Parents process*/
        clusterId++;
      }
    }
  //wait for all child
  waitpid(-1, NULL, 0);
}

qemuBuilder::~qemuBuilder()
{
  delete[] clusters;
}
