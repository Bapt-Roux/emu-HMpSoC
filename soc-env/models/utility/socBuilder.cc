#include "socBuilder.h"

/*---
 * CLASS
 * METHODS IMPLEMENTATION
 *----------------------------------------------------------------------------*/
using namespace sb;
// using namespace ms;
using namespace std;

SoCBuilder::SoCBuilder(ifstream & stream, char stype)
{
  char nocIpcName[64];
  cereal::importConfig<socCnf>(topLevel, stream, stype);

  /*---
   * NOC ROUTER SETUP
   * --------------------------------------------------*/
  noc::nocRouter_base *router;
  if (topLevel.global.multiThread_sim){ //use noc::ipc implementation
    sprintf(nocIpcName, "%s/%s", topLevel.global.sk_bpath.c_str(), topLevel.global.sk_noc.c_str());
    router = new noc::ipc::nocRouter(topLevel.clusters.sMap.size(), nocIpcName, topLevel.noc);
  }else{//use noc::mono implementation
    router = new noc::mono::nocRouter("noc_router", topLevel.clusters.sMap.size(), topLevel.noc);
  }

  /*--- extract discovery informations                 */
  noc::clPos cl_pos;
  cl_pos.sk = 0xFF;
  for( auto & it_NoC: topLevel.clusters.sMap){
    cl_pos.x_pos = it_NoC.second.x_pos;
    cl_pos.y_pos = it_NoC.second.y_pos;
    nocDiscover[it_NoC.second.gb_addr] = cl_pos;
  }
#ifdef SB_DEBUG
  std::cout << "NoC configuration read from socCnf is:\n";
  for( auto & it_NoC: nocDiscover){
    std::cout << " @" <<std::hex<< it_NoC.first << "\n"
              << "\t Sk: " <<std::dec<< it_NoC.second.sk
              << " =>["<<(int)it_NoC.second.x_pos
              << ","<<(int)it_NoC.second.y_pos << "]\n";

  }
#endif

  /*---
   * CLUSTERS SETUP
   * --------------------------------------------------*/
  if (topLevel.global.multiThread_sim){ //Configure simulation within multi-process and multi-systemC simulation kernel
    //allocate space for clusters pid
    clusters_pid = new pid_t[topLevel.clusters.sMap.size()];
    uint clusterId=0;
    for (const auto & it_cluster: topLevel.clusters.sMap)
      {
        // spawn cluster Process
        clusters_pid[clusterId] = fork();
        if(0 == clusters_pid[clusterId]){
          sc_set_time_resolution(1, SC_NS);
          ClusterBuilder sc_Cluster(it_cluster.first.c_str(), clusterId, topLevel.global,
                                    it_cluster.second, nocDiscover);
          sc_Cluster.rst.write(true);
          sc_start(1, SC_US);
          sc_Cluster.rst.write(false);
          sc_start();
          cout << "Cluster["<<clusterId <<"] process terminated"<< endl;
          exit(1);
        }else if(0> clusters_pid[clusterId]) {
          cerr << "Fork failed for Cluster: "<< it_cluster.first << ", Id: " <<clusterId << endl;
          exit(1);
        }else{ /*Parents process*/
          router->bindToCluster(NULL, NULL,
                                it_cluster.second.gb_addr, it_cluster.second.x_pos, it_cluster.second.y_pos);
          clusterId++;}
      }

    router->startRouter();
    waitpid(-1,NULL,0); //any child
  }else{//Configure simulation within one systemC simulation kernel
    clusters_sc =  new ClusterBuilder*[topLevel.clusters.sMap.size()];
    uint clusterId=0;
    uint trgtSk=0;
    for (const auto & it_cluster: topLevel.clusters.sMap){
      clusters_sc[clusterId] = new ClusterBuilder(it_cluster.first.c_str(), clusterId, topLevel.global,
                                            it_cluster.second, nocDiscover);
      trgtSk = router->bindToCluster(clusters_sc[clusterId]->iNoC->get_noc_Ssk(),
                                     clusters_sc[clusterId]->iNoC->get_noc_Msk(),
                                     it_cluster.second.gb_addr, it_cluster.second.x_pos, it_cluster.second.y_pos);
      clusters_sc[clusterId]->rst.write(true);
      clusterId++;
    }
    sc_start(1, SC_US);
    for(clusterId=0; clusterId< topLevel.clusters.sMap.size(); clusterId++){
      clusters_sc[clusterId]->rst.write(false);
    }
    sc_start();
  }
}

SoCBuilder::~SoCBuilder()
{
  if (topLevel.global.multiThread_sim){
    delete[] clusters_pid;
  }else{
    uint clusterId;
    for(clusterId=0; clusterId< topLevel.clusters.sMap.size(); clusterId++)
      delete clusters_sc[clusterId];
    delete[] clusters_sc;
  }
}

ClusterBuilder::ClusterBuilder(sc_core::sc_module_name name, uint id, generalCnf global, clusterCnf clusterConfig,
                               std::map<uint, noc::clPos>nocDiscover)
  : sc_module(name), to_zynq_sk("tozynq")
  ,clusterLevel(clusterConfig), clusterId(id)
{
  /*---
   * CLUSTERS SETUP
   * --------------------------------------------------*/
#ifdef SB_DEBUG
  cout << "Begin generation of " << name << "\n";
#endif

  /*---
   * MONSYSTEM SETUP
   * ---------------------------------------- */
  uint nbSlaves=0;
  uint msSlaveSocket =0;
  ms::clusterProp msProps;

  uint8_t amba_subCpn = clusterLevel.ambaGP.hwCpn.sMap.size();
  uint ms_Slave = (amba_subCpn+2);
  msProps.ms_addr= clusterLevel.ms_addr;
  msProps.memory.comChan = clusterLevel.memory.comChan;
  msProps.amba.comChan = clusterLevel.ambaGP.comChan;
  for (const auto & it_hw: clusterLevel.ambaGP.hwCpn.sMap){
    ms::blockProp tmpBlk;
    tmpBlk.comChan = it_hw.second.comChan;
    msProps.hwCpn.sMap[it_hw.first] = tmpBlk;
  }
  monitor = new ms::monSysMaster("monitor", ms_Slave, msProps);

  //prerequisite for vars declaration
  int ic_sublevel= 5; //TODO add condition to switch to 3 in case of NoC

  /*---
   * CPU SETUP
   * ---------------------------------------- */
  // char qemuIpcName[64];
  cpuCnf cpuConf = clusterLevel.cpu;
  std::string qemuIpcName ="unix:"+global.sk_bpath+"/cl@"+ to_string(clusterId)+"/"+ global.sk_qemu+"0";
  //libremote keep only string@ so we need to enhance string scop
  sk_descr = new char[qemuIpcName.size()*sizeof(char)];
  strcpy(sk_descr, qemuIpcName.c_str());
  cpu = new xilinx_zynq("cpu", sk_descr);
  m_qk.set_global_quantum(sc_time(1, SC_US));
  cpu->s_axi_gp[0]->bind(to_zynq_sk);
  cpu->rst(rst);

  /*---
   * INTERCONNECT SETUP
   * ---------------------------------------- */
  //TODO change iconnect component to generate error when more than ic_sublevel try to connect ATM silently failed
  bus = new iconnect("bus", 1, ic_sublevel);
  cpu->m_axi_gp[0]->bind(*(bus->cpu_Ssk[0]));
  //map monSysMaster master socket in memory
  bus->memmap(clusterLevel.ms_addr, monitor->getAddrSpace(),
              ADDRMODE_RELATIVE, -1, (monitor->cpu_Ssk)); //TODO set 3,4 params user setable

  /*---
   * NoC Itf SETUP
   * ---------------------------------------- */
  //Mark local cluster in nocDiscovery
  std::map<uint, noc::clPos> nocTmp = nocDiscover;
  nocTmp.erase(clusterConfig.gb_addr);
  noc::clPos cl_pos;
  cl_pos.x_pos = clusterConfig.x_pos;
  cl_pos.y_pos = clusterConfig.y_pos;
  nocTmp[clusterConfig.gb_addr +0xffff] = cl_pos;

  if (global.multiThread_sim){
    char nocIpcName[64];
    sprintf(nocIpcName, "%s/%s", global.sk_bpath.c_str(), global.sk_noc.c_str());
    //compute noc sync postBoot_us such as 10 cpuSync occured between nocSync
    uint postBoot_us = (global.sync_quantum/100);
    iNoC = new noc::ipc::nocItf("nocItf", nocTmp, global.preBoot_ms, postBoot_us);
    iNoC->openUnixIpc(nocIpcName);
  }else{
    iNoC = new noc::mono::nocItf("nocItf", nocTmp);
  }
  bus->memmap(clusterLevel.noc_addr, RNOC_POOLSIZE ,
              ADDRMODE_RELATIVE, -1, iNoC->cmd_Ssk);//TODO set 3,4 params user setable
  iNoC->irq_wakeUp(cpu->pl2ps_irq[0]);
  iNoC->irq_orderRcv(cpu->pl2ps_irq[1]);

  /*---
   * MEMORY SETUP
   * ---------------------------------------- */
  memoryCnf memoryConf= clusterLevel.memory;
  sc_time latency = sc_time(memoryConf.latency_ns,SC_NS);
  ram = new memory("ram", memoryConf.HPx, latency, memoryConf.addr_space, global.fast_dbg);
  bus->memmap(memoryConf.addr_offset, memoryConf.addr_space,
              ADDRMODE_RELATIVE, -1, ram->cpu_Ssk); //TODO set 3,4 params user setable
  // cpu->map_ram("ram", //FIXME
  //              memoryConf.addr_offset, memoryConf.addr_space, 1); //TODO set params 4 user setable
  //link memory to iNoC
  iNoC->DMA_Msk.bind(ram->noc_Ssk);
  //link MonSysSlave to Master
  monitor->ip_Msk[msSlaveSocket++]->bind(ram->ms_Ssk);

  /*---
   * AMBA GP SETUP
   * ---------------------------------------- */
  ambaCnf ambaConf = clusterLevel.ambaGP;
  amba_gp = new amba_GP("amba", amba_subCpn, ambaConf.hwCpn);
  bus->memmap(ambaConf.addr_offset, ambaConf.addr_space,
              ADDRMODE_RELATIVE, -1, amba_gp->cpu_Ssk);//TODO set 3,4 params user setable
  //link MonSysSlave to Master
  monitor->ip_Msk[msSlaveSocket++]->bind(amba_gp->ms_Ssk);

  int hw_id=0;
  int hpx_offset=0;
  int irq_offset=2; //Two first irq_lines reserved for nocItf
  /* Hw components setup */
  hw = new hwIP*[amba_subCpn];
  for (const auto & it_hw: ambaConf.hwCpn.sMap)
    {
      hwCpnCnf hwConf = it_hw.second;
#ifdef SB_DEBUG
      cout << "Begin generation of hw["<< hw_id << "] " << it_hw.first
           << "[T."<<hwConf.ipType <<"]"<< "\n";
#endif

      if ("aes" == hwConf.ipType){
        hw[hw_id] = (hwIP*) new aes_hwStub(it_hw.first.c_str(), hwConf.compTime_ns, hwConf.compEn_nj);
      }
      else if ("viterbi" == hwConf.ipType){
        hw[hw_id] = (hwIP*) new viterbi_hwStub(it_hw.first.c_str(), hwConf.compTime_ns, hwConf.compEn_nj);
      }
      else if ("bfs_bulk" == hwConf.ipType){
        hw[hw_id] = (hwIP*) new bfs_bulk_hwStub(it_hw.first.c_str(), hwConf.compTime_ns, hwConf.compEn_nj);
      }else if ("backprop" == hwConf.ipType){
        hw[hw_id] = (hwIP*) new backprop_hwStub(it_hw.first.c_str(), hwConf.compTime_ns, hwConf.compEn_nj);
      }else if ("fft_strided" == hwConf.ipType){
        hw[hw_id] = (hwIP*) new fft_strided_hwStub(it_hw.first.c_str(), hwConf.compTime_ns, hwConf.compEn_nj);
      }else if ("gemm_blocked" == hwConf.ipType){
        hw[hw_id] = (hwIP*) new gemm_blocked_hwStub(it_hw.first.c_str(), hwConf.compTime_ns, hwConf.compEn_nj);
      }else if ("kmp" == hwConf.ipType){
        hw[hw_id] = (hwIP*) new kmp_hwStub(it_hw.first.c_str(), hwConf.compTime_ns, hwConf.compEn_nj);
      }else if ("md_knn" == hwConf.ipType){
        hw[hw_id] = (hwIP*) new md_knn_hwStub(it_hw.first.c_str(), hwConf.compTime_ns, hwConf.compEn_nj);
      }else if ("nw" == hwConf.ipType){
        hw[hw_id] = (hwIP*) new nw_hwStub(it_hw.first.c_str(), hwConf.compTime_ns, hwConf.compEn_nj);
      }else if ("sort_merge" == hwConf.ipType){
        hw[hw_id] = (hwIP*) new sort_merge_hwStub(it_hw.first.c_str(), hwConf.compTime_ns, hwConf.compEn_nj);
      }else if ("stenc2d" == hwConf.ipType){
        hw[hw_id] = (hwIP*) new stenc2d_hwStub(it_hw.first.c_str(), hwConf.compTime_ns, hwConf.compEn_nj);
      }else if ("spmv_crs" == hwConf.ipType){
        hw[hw_id] = (hwIP*) new spmv_crs_hwStub(it_hw.first.c_str(), hwConf.compTime_ns, hwConf.compEn_nj);
      }

      //link to amba bridge
      amba_gp->pl_Msk[hw_id]->bind(hw[hw_id]->amba_Ssk);
      hw[hw_id]->dma_Msk[0]->bind((*ram->HPx_Ssk[hwConf.hpx_id + hpx_offset]));
      hw[hw_id]->irqn[0](cpu->pl2ps_irq[hwConf.irq_id + irq_offset]);

      //link MonSysSlave to Master
      monitor->ip_Msk[msSlaveSocket++]->bind(hw[hw_id]->ms_Ssk);
      hw_id++;
    }

  /*---
   * SC getTime setup
   * ---------------------------------------- */
  getTime = new scTime("scTime");
  bus->memmap(clusterLevel.gt_addr, SCTIME_RSIZE,
              ADDRMODE_RELATIVE, -1, getTime->time_Ssk); //TODO set 3,4 params user setable

  //tie unconnected cpu 0 to prevent sysc error
  cpu->tie_off();
  //send default channels params to msSlave
  monitor->monSysInit();

  /*---
   * SC tracing SETUP
   * ---------------------------------------- */
  if(global.run_trace){
    string traceName = "trace_@cl"+to_string(clusterId);
    tracer = new scTrace(traceName.c_str());
    tracer->traceModule(*cpu, cpu->name());
    tracer->traceModule(*iNoC, iNoC->name());
  }


#ifdef SB_DEBUG
  //Save monSystemConfig
  ofstream  msFile;
  stringstream fileName;
  fileName <<"msConf_"<<name <<".json";
  msFile.open(fileName.str());
  monitor->exportClProp(msFile, 'j');
  msFile.close();
  cout << "End of " << name << "\n";
#endif
}

ClusterBuilder::~ClusterBuilder()
{
  delete[] sk_descr;
}
