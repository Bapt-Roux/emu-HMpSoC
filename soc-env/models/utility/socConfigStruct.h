/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: socConfigStruct.h
 * Configuration structure with serialization feature for SoC configuration
 */

#ifndef _SOC_CONFIG_STRUCT
#define _SOC_CONFIG_STRUCT

// cereal header extension
#include "models/utility/cerealExt.h"
#include "models/monSystem/msConfigStruct.h"

/*---
 *  Configuration structure for SoC configuration
 *----------------------------------------------------------------------------*/
namespace sb{
  struct hwCpnCnf{
    //properties
    std::string ipType;
    uint hpx_id;
    uint irq_id;
    ulong addr_offset;
    uint addr_space;
    uint compTime_ns;
    uint compEn_nj;
    cereal::serialMap<ms::params> comChan;

    template<class Archive>	void save(Archive & ar) const{
      ar(CEREAL_NVP(ipType));
      ar(CEREAL_NVP(hpx_id));
      ar(CEREAL_NVP(irq_id));
      saveHex(ar, "addr_offset", addr_offset);
      saveHex(ar, "addr_space", addr_space);
      ar(CEREAL_NVP(compTime_ns));
      ar(CEREAL_NVP(compEn_nj));
      ar(CEREAL_NVP(comChan));
    }
    template<class Archive>	void load(Archive & ar){
      ar(CEREAL_NVP(ipType));
      ar(CEREAL_NVP(hpx_id));
      ar(CEREAL_NVP(irq_id));
      addr_offset = getHex(ar);
      addr_space = getHex(ar);
      ar(CEREAL_NVP(compTime_ns));
      ar(CEREAL_NVP(compEn_nj));
      ar(CEREAL_NVP(comChan));
    }
  };

  struct ambaCnf{
    //properties
    ulong addr_offset;
    uint addr_space;
    cereal::serialMap<ms::params> comChan;
    //ipLevel
    cereal::serialMap<hwCpnCnf> hwCpn;

    template<class Archive>	void save(Archive & ar) const{
      saveHex(ar, "addr_offset", addr_offset);
      saveHex(ar, "addr_space", addr_space);
      ar(CEREAL_NVP(comChan));
      ar(CEREAL_NVP(hwCpn));
    }
    template<class Archive>	void load(Archive & ar){
      addr_offset = getHex(ar);
      addr_space = getHex(ar);
      ar(CEREAL_NVP(comChan));
      ar(CEREAL_NVP(hwCpn));
    }
  };

  struct memoryCnf{
    //properties
    ulong addr_offset;
    uint addr_space;
    uint latency_ns;
    uint HPx;
    cereal::serialMap<ms::params> comChan;

    template<class Archive>	void save(Archive & ar) const{
      saveHex(ar, "addr_offset", addr_offset);
      saveHex(ar, "addr_space", addr_space);
      ar(CEREAL_NVP(latency_ns));
      ar(CEREAL_NVP(HPx));
      ar(CEREAL_NVP(comChan));
    }
    template<class Archive>	void load(Archive & ar){
      addr_offset = getHex(ar);
      addr_space = getHex(ar);
      ar(CEREAL_NVP(latency_ns));
      ar(CEREAL_NVP(HPx));
      ar(CEREAL_NVP(comChan));
    }
  };

  struct cpuCnf{
    std::string dtb_name;

    template<class Archive>	void serialize(Archive & ar){
      ar(CEREAL_NVP(dtb_name));
    }
  };


  struct clusterCnf{
    ulong gb_addr;
    ulong ms_addr;
    ulong gt_addr;
    ulong noc_addr;
    uint8_t x_pos;
    uint8_t y_pos;
    cpuCnf cpu;
    memoryCnf memory;
    ambaCnf ambaGP;

    template<class Archive>	void save(Archive & ar) const{
      saveHex(ar, "gb_addr", gb_addr);
      saveHex(ar, "ms_addr", ms_addr);
      saveHex(ar, "gt_addr", gt_addr);
      saveHex(ar, "noc_addr", noc_addr);
      ar(CEREAL_NVP(x_pos));
      ar(CEREAL_NVP(y_pos));
      ar(CEREAL_NVP(cpu));
      ar(CEREAL_NVP(memory));
      ar(CEREAL_NVP(ambaGP));
    }
    template<class Archive>	void load(Archive & ar){
      gb_addr   = getHex(ar);
      ms_addr   = getHex(ar);
      gt_addr   = getHex(ar);
      noc_addr  = getHex(ar);
      ar(CEREAL_NVP(x_pos));
      ar(CEREAL_NVP(y_pos));
      ar(CEREAL_NVP(cpu));
      ar(CEREAL_NVP(memory));
      ar(CEREAL_NVP(ambaGP));
    }
  };

  /***
   * Noc configuration
   *******************************/
  enum nocType {RING, MESH_2D, TORUS_2D};
  static const std::map<nocType,std::string> nocType_helper = {{RING,"RING"},{MESH_2D,"MESH_2D"},{TORUS_2D,"TORUS_2D"}};

  struct nocCnf{
    nocType topo;
    uint8_t xSize, ySize;
    ms::nocProp params;

    template<class Archive>	void save(Archive & ar) const{
      ar(cereal::make_nvp("topology", nocType_helper.find(topo)->second)); //TODO catch error if bad topo value
      ar(CEREAL_NVP(xSize));
      ar(CEREAL_NVP(ySize));
      ar(CEREAL_NVP(params));
    }
    template<class Archive>	void load(Archive & ar){
      std::string strBuffer;
      ar(strBuffer);
        topo= (nocType)0; // default in case of error in string config
      for (const auto it: nocType_helper){
        if(it.second == strBuffer){
          topo = it.first;
          break;
        }
      }
      ar(CEREAL_NVP(xSize));
      ar(CEREAL_NVP(ySize));
      ar(CEREAL_NVP(params));
    }
  };

  struct generalCnf{
    std::string sk_bpath;
    std::string sk_qemu;
    std::string sk_noc;
    uint ssh_port_offset;
    uint gdb_port_offset;
    uint sync_quantum;
    uint sync_icount;
    uint preBoot_ms;
    bool multiThread_sim;
    bool run_trace;
    bool fast_dbg;

    template<class Archive>	void serialize(Archive & ar){
      ar(CEREAL_NVP(sk_bpath));
      ar(CEREAL_NVP(sk_qemu));
      ar(CEREAL_NVP(sk_noc));
      ar(CEREAL_NVP(ssh_port_offset));
      ar(CEREAL_NVP(gdb_port_offset));
      ar(CEREAL_NVP(sync_quantum));
      ar(CEREAL_NVP(sync_icount));
      ar(CEREAL_NVP(preBoot_ms));
      ar(CEREAL_NVP(multiThread_sim));
      ar(CEREAL_NVP(run_trace));
      ar(CEREAL_NVP(fast_dbg));
    }
  };


  struct socCnf{
    generalCnf global;
    nocCnf noc;
    cereal::serialMap<clusterCnf> clusters;

    template<class Archive>	void serialize(Archive & ar){
      ar(CEREAL_NVP(global));
      ar(CEREAL_NVP(noc));
      ar(CEREAL_NVP(clusters));
    }
  };

}// end namespace sb

#endif /*_SOC_CONFIG_STRUCT*/
