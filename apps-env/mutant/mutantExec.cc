/*
 * Copyright (c) 2017 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/**
 * @file: mutantExec.cc
 * @brief Execution stub for a given mutant graph
 */

#include "mutantExec.h"
using namespace napi;
using namespace clmgmt;


void usage(void)
{
  std::cout << "argv[0] <GRAPH_Filename> \n";
  return;
}

int main(int argc, char ** argv)
{
  std::string graphPath;
  if(1> argc){
    std::cout << "Error: No GRAPH_Filename given./n";
    usage();
    return -1;
  }else{
    graphPath = argv[1];
  }

  //get application graph properties
  mutantGen appGraph(graphPath);

  // Init usefull API
  clMaster mgmtMaster(NOC_DEVICE);
#ifdef MEXEC_DEBUG
  const std::map<uint,noc::clPos> confCl = mgmtMaster.getNocApi().getClusters();
  std::cout << "NoC configuration read from Master cluster is:\n";
  for( auto & it_NoC: confCl){
    std::cout << "\t @" <<std::hex<< it_NoC.first
              << " =>["<<(int)it_NoC.second.x_pos
              << ","<<(int)it_NoC.second.y_pos << "]\n";
  }
#endif

  /* Spawn task starting by end *********************************************/
  nodes *cur_node;

  for(int d=(appGraph.getDepth() -1); d >= 0; d--){
    for(uint w=0; w< appGraph.getWidth(); w++){
      std::cout<< "Current NODES: ["<<std::dec<< w <<"; "<<d<<"] ============================================\n\n";
      noc::task_t trgt_uid;
      uint64_t trgt_cl;
      size_t len_dataIn, len_dataOut;

      // Get nodes properties
      cur_node = appGraph.getNode(w, d);
      if (NULL == cur_node){ continue;}
#ifdef MEXEC_DEBUG
      std::cout << "task_id: 0x" <<std::hex<< cur_node->task_id <<"\n";
      std::cout << "nbIteration: 0x" << (uint)cur_node->nbIteration <<"\n";
      std::cout << "id: 0x" << (uint)cur_node->id <<"\n";
      std::cout << "clTarget: 0x" << (uint)cur_node->clTarget <<"\n";
      std::cout << "hwTask: " << (cur_node->hwTask? "True":"False") <<"\n";
#endif
      trgt_uid = getTaskUID(cur_node->task_id, cur_node->hwTask, cur_node->id);
      trgt_cl = (((uint64_t)cur_node->clTarget)*CL_OFFSET_ADDR) + CL_BASE_ADDR;
      trgtId_t trgt = std::make_pair(trgt_uid, trgt_cl);
      (machSuiteDataSize.at(trgt_uid& (NOC_JOBMASK^HWFLAG)))(cur_node->nbIteration, len_dataIn, len_dataOut);

      // Setup providers & receivers
      std::deque<comProps> providers, receivers;
      comProps tmpRcv, tmpPrv;
      std::vector<std::pair<noc::task_t, uint64_t>> rcvIt;
      std::vector<std::tuple<noc::task_t, size_t, uint64_t>> prvIt;
      prvIt = getProviders(&appGraph, w, d); // Providers
      for(const auto &it: prvIt){
        tmpPrv.task = std::get<0>(it);
        tmpPrv.dSize = std::get<1>(it);
        tmpPrv.clAddr = std::get<2>(it);
        providers.push_back(tmpPrv);
      }
      rcvIt = getReceivers(&appGraph, w, d); // Receivers
      for(const auto &it: rcvIt){
        tmpRcv.task = it.first;
        tmpRcv.dSize = len_dataOut;
        tmpRcv.clAddr = it.second;
        receivers.push_back(tmpRcv);
      }
      // Ask slave and send Providers&Receivers properties
      mgmtMaster.askSlaveSubscribe(trgt);
      mgmtMaster.setProviderAndReceiver(trgt, providers, receivers);
    }
  }

  std::cout << "End execution mutant GRAPH -----------------------\n";

  return(EXIT_SUCCESS);
}

/**
 * @brief Return task unique ID from node properties
 * @param task: task as jobId
 * @param hw: hw implementation
 * @param id: unique id
 */
noc::task_t getTaskUID(uint task, bool hw, uint id){
  noc::task_t task_uid = ((task << TASK_TO_TID | (true==hw?HWFLAG:0)) & NOC_JOBMASK); //job id
  task_uid |=  ((id << TASK_IDOFFSET) & (NOC_JOBMASK^NOC_TASKMASK)); //job id
  return(task_uid);
}

/**
 * @brief Return List of receivers nodes
 * @param graph: mutantGen application graph
 * @param x_pos: position target node (width)
 * @param y_pos: position target node (depth)
 */
std::vector<std::pair<noc::task_t, uint64_t>> getReceivers(mutantGen *graph, uint x_pos, uint y_pos){
  std::vector<std::pair<noc::task_t, uint64_t>> list;
  nodes *cur_node;
  noc::task_t task_uid;

  for( uint d = (y_pos+1); (d<=(y_pos +graph->getEdgesLen())) && (d< graph->getDepth()); d++){
    for( uint w = 0; w<graph->getWidth() ; w++){
      print_DMEXEC(" Search Receivers at x:" << std::dec << w << "; y: " << d << "\n");
      if( 1 == *graph->getOutEdge(x_pos, y_pos, w, d)){
        cur_node = graph->getNode(w, d);
#ifdef MEXEC_DEBUG
        std::cout << "Found receivers target node \n";
        std::cout << "task_id: 0x" <<std::hex<< cur_node->task_id <<"\n";
        std::cout << "nbIteration: 0x" << (uint)cur_node->nbIteration <<"\n";
        std::cout << "id: 0x" << (uint)cur_node->id <<"\n";
        std::cout << "clTarget: 0x" << (uint)cur_node->clTarget <<"\n";
        std::cout << "hwTask: " << (cur_node->hwTask? "True":"False") <<"\n";
#endif
        task_uid = getTaskUID(cur_node->task_id, cur_node->hwTask, cur_node->id);
        list.push_back(std::make_pair(task_uid, (((uint64_t)cur_node->clTarget)*CL_OFFSET_ADDR) + CL_BASE_ADDR));
      }
    }
  }

  return(list);
}

/**
 * @brief Return List of providers nodes
 * @param graph: mutantGen application graph
 * @param x_pos: position target node (width)
 * @param y_pos: position target node (depth)
 */
std::vector<std::tuple<noc::task_t, size_t, uint64_t>> getProviders(mutantGen *graph, uint x_pos, uint y_pos){
  std::vector<std::tuple<noc::task_t, size_t, uint64_t>> list;
  nodes *cur_node;
  noc::task_t task_uid;

  int start = y_pos-graph->getEdgesLen();
  start = (0> start)?0:start;

  for( uint d = start; d < y_pos ; d++){
    for( uint w = 0; w<graph->getWidth() ; w++){
      print_DMEXEC(" Search Providers at x:" << std::dec << w << "; y: " << d << "\n");
      if( 1 == *graph->getInEdge(x_pos, y_pos, w, d)){
        cur_node = graph->getNode(w, d);
#ifdef MEXEC_DEBUG
        std::cout << "Found providers target node \n";
        std::cout << "task_id: 0x" <<std::hex<< cur_node->task_id <<"\n";
        std::cout << "nbIteration: 0x" << (uint)cur_node->nbIteration <<"\n";
        std::cout << "id: 0x" << (uint)cur_node->id <<"\n";
        std::cout << "clTarget: 0x" << (uint)cur_node->clTarget <<"\n";
        std::cout << "hwTask: " << (cur_node->hwTask? "True":"False") <<"\n";
#endif
        task_uid = getTaskUID(cur_node->task_id, cur_node->hwTask, cur_node->id);
        size_t prvIn, prvOut;
        (machSuiteDataSize.at(task_uid & (NOC_JOBMASK^HWFLAG)))(cur_node->nbIteration, prvIn, prvOut);
        list.push_back(std::make_tuple(task_uid, prvOut, (((uint64_t)cur_node->clTarget)*CL_OFFSET_ADDR) + CL_BASE_ADDR));
      }
    }
  }

  return(list);
}
