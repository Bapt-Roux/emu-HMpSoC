/**
 * @file md_knn_hws.h
 * @brief Header file for emu-hmpsoc MD_KNN HW stub.
 */
#ifndef _MD_KNN_HWS
#define _MD_KNN_HWS

#include <models/fpga/hwIP.h>
#include <md_knn.h>

/**
 * @class md_knn_hwStub
 * @brief integrated the md_knn computation function within a systemC Ip.
 *----------------------------------------------------------------------------*/
class md_knn_hwStub: public hwIP
{
  uint cTime_ns, cEn_nj;
  void parse_cmd();

  TYPE_MDK xp[nAtoms], yp[nAtoms], zp[nAtoms];
  int32_t neighbors[nAtoms*maxNeighbors];
  TYPE_MDK xf[nAtoms], yf[nAtoms], zf[nAtoms];

public:
  SC_HAS_PROCESS(md_knn_hwStub);
  md_knn_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj);
  ~md_knn_hwStub();
};

#endif /*_MD_KNN_HWS*/
