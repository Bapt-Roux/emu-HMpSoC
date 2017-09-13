/**
 * @file mutantGraph.h
 * @brief Header file for mutant graph structure
 */
#ifndef _MUTANTGRAPH
#define _MUTANTGRAPH

#ifdef MGRAPH_DEBUG
#   define print_DMGRAPH(message)                           \
  do {std::cout << message;}                              \
  while (false)
#define assert(cond, message) \
  do{ if(!(cond)){ std::cout<< __PRETTY_FUNCTION__<<"Assertion failure: "<< message<< std::endl;}} \
  while(false)
#else
#   define print_DMGRAPH(message)                 \
  do {} while (false)
#define assert(cond, message)                                           \
  do{} while(false)
#endif


//2D mtx index
#define _2d_nodes(i,j) ((i)*(width) + (j))
// #define _2d_edges(i,j) ((i)*(width*depth) + (j))
// edges connecting  i,j to node ti,tj
#define _2d_toEdges(i,j,ti,tj) (((ti)*width +(tj))*(width*depth) +((i)*width + (j)))

#define _2d_getEdges(i,j,ti,tj) (((i)*width +(j))*(width*depth) +((ti)*width + (tj)))

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <vector>

struct nodes
{
  /// Job properties
  std::string task_name;
  uint8_t nbIteration;
  uint8_t id;
  /// Mapping properties
  uint8_t clTarget;
  bool hwTask;
};


class mutantGraph
{
  /// Grapheral properties
  uint depth, width;
  uint nbNodes;
  uint edgesDensity, edgesLength;

  /// Graph descriptors
  nodes ** mtx_nodes;
  uint * mtx_edges;

  /// Methodes
  void memorySetup();

  /// Debug
  void rawDisplay();
public:
  mutantGraph(uint _depth=0, uint _width=0, uint _nodes=0,
            uint _eDensity=1, uint _eLength=1);
  mutantGraph(std::string file);
  ~mutantGraph();

  /// Load & Store
  void save(std::string file);
  void load(std::string file);

  /// Getter
  uint getDepth(){return depth;};
  uint getWidth(){return width;};
  uint getNbNodes(){return(nbNodes);};
  uint getDensity(){return(edgesDensity);};
  uint getEdgeLen(){return(edgesLength);};
  nodes getNode(uint x_pos, uint y_pos){return( (*mtx_nodes[_2d_nodes(x_pos, y_pos)]));};
  uint getEdge(uint x_start, uint y_start, uint x_dest, uint y_dest){
    return(mtx_edges[_2d_toEdges(x_start, y_start, x_dest, y_dest)]);};

  /// Setter
  void updtNode(nodes props, uint x_pos, uint y_pos);
  void updtNodeId(uint id, uint x_pos, uint y_pos);
  void updtEdge(uint val, uint x_start, uint y_start, uint x_dest, uint y_dest);

};

#endif /*_MUTANTGRAPH*/
