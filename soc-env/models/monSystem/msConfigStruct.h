/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: msConfigStruct.h
 * Configuration structure with serialization feature for configuration of Monitoring system
 */

#ifndef _MS_CONFIG_STRUCT
#define _MS_CONFIG_STRUCT

//types
#include <cereal/types/map.hpp>
//archives format
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"

/*---
 * MonitorSystem structure definition (property for one cluster)
 *----------------------------------------------------------------------------*/
namespace ms{
  /*---
   * Channel Config structure definition
   *-------------------------------------------*/
  struct params{
    size_t maxBurst;
    float engS;
    float engD;
    uint timeS_ns;
    uint timeD_ns;

    template<class Archive>	void serialize(Archive & ar){
      ar(CEREAL_NVP(maxBurst));
      ar(CEREAL_NVP(engS));
      ar(CEREAL_NVP(engD));
      ar(CEREAL_NVP(timeS_ns));
      ar(CEREAL_NVP(timeD_ns));
    }
  };

  struct blockProp{
    ulong blk_addr;
    cereal::serialMap<params> comChan;

    template<class Archive>	void save(Archive & ar) const{
      saveHex(ar, "blk_addr", blk_addr);
      ar(CEREAL_NVP(comChan));
    }
    template<class Archive>	void load(Archive & ar){
      blk_addr = getHex(ar);
      ar(CEREAL_NVP(comChan));
    }
  };

  struct clusterProp{
    ulong ms_addr;
    blockProp memory;
    blockProp amba;
    cereal::serialMap<blockProp> hwCpn;

    template<class Archive>	void save(Archive & ar) const{
      saveHex(ar, "ms_addr", ms_addr);
      ar(CEREAL_NVP(memory));
      ar(CEREAL_NVP(amba));
      ar(CEREAL_NVP(hwCpn));
    }
    template<class Archive>	void load(Archive & ar){
      ms_addr = getHex(ar);
      ar(CEREAL_NVP(memory));
      ar(CEREAL_NVP(amba));
      ar(CEREAL_NVP(hwCpn));
    }
  };

  struct nocProp{
    size_t maxPacketLength;
    //time params in NS
    uint tns_atm; //accessToMedia
    uint tns_rcc; //route calculation cost
    uint tns_sbc; //segment byte cost
    //Energy params in Joule
    float enj_atm; //accessToMedia
    float enj_rcc; //route calculation cost
    float enj_sbc; //segment byte cost

    template<class Archive>	void serialize(Archive & ar){
      ar(CEREAL_NVP(maxPacketLength));
      ar(CEREAL_NVP(tns_atm));
      ar(CEREAL_NVP(tns_rcc));
      ar(CEREAL_NVP(tns_sbc));
      ar(CEREAL_NVP(enj_atm));
      ar(CEREAL_NVP(enj_rcc));
      ar(CEREAL_NVP(enj_sbc));
    }
  };

}// end namespace ms

#endif /*_MS_CONFIG_STRUCT*/
