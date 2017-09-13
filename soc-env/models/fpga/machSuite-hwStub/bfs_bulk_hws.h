/**
 * @file bfs_bulk_hws.h
 * @brief Header file for emu-hmpsoc BFS_BULK HW stub.
 */
#ifndef _BFS_BULK_HWS
#define _BFS_BULK_HWS

#include <models/fpga/hwIP.h>
#include <bfs_bulk.h>

/**
 * @class bfs_bulk_hwStub
 * @brief integrated the bfs_bulk computation function within a systemC Ip.
 *----------------------------------------------------------------------------*/
class bfs_bulk_hwStub: public hwIP
{
  uint cTime_ns, cEn_nj;
  void parse_cmd();
  node_index_t starting_node;
  node_t nodes[N_NODES];
  edge_t edges[N_EDGES];
  edge_index_t loc_lvlCnts[N_LEVELS];
  level_t lvl[N_NODES];


public:
  SC_HAS_PROCESS(bfs_bulk_hwStub);
  bfs_bulk_hwStub(sc_core::sc_module_name name, uint compTime_ns, uint compEn_nj);
  ~bfs_bulk_hwStub();
};

#endif /*_BFS_BULK_HWS*/
