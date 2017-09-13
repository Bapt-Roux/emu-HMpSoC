/**
 * @file mutantGen.h
 * @brief Header file for application mutant generator
 */
#ifndef _MUTANTGEN
#define _MUTANTGEN

#define MGEN_DEBUG
#ifdef MGEN_DEBUG
#   define print_DMGEN(message)                           \
  do {std::cout << message;}                              \
  while (false)
#define assert(cond, message) \
  do{ if(!(cond)){ std::cout<< __PRETTY_FUNCTION__<<"Assertion failure: "<< message<< std::endl;}} \
  while(false)
#else
#   define print_DMGEN(message)                 \
  do {} while (false)
#define assert(cond, message)                                           \
  do{} while(false)
#endif

//2D mtx index
#define _2d_node(x, y) ((x) +(y)*(width))
#define _outEdges(Xs, Ys, Xt, Yt)((width*(Ys)+ (Xs)) + (width*depth)*(width*(Yt) +(Xt)))
#define _inEdges(Xs, Ys, Xt, Yt)((width*depth)*(width*(Ys)+ (Xs)) + (width*(Yt) +(Xt)))

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
  uint task_id;
  uint8_t nbIteration;
  uint8_t id;
  /// Mapping properties
  uint8_t clTarget;
  bool hwTask;
};


class mutantGen
{
  /// General properties
  uint depth, width;
  uint nbNodes;
  uint edgesDensity, edgesLength;
  uint nbCluster, maxIter;

  /// Graph descriptors
  nodes ** mtx_nodes;
  uint * mtx_edges;

  /// Methodes
  void memorySetup();
  void randomGen(std::vector<uint>bench, bool randMap);
public:
  mutantGen(std::string filepath);
  mutantGen(uint _depth=0, uint _width=0, uint _nodes=0,
            uint _eDensity=1, uint _eLength=1,
            uint _nbCluster=2, uint _maxIter=1,
            std::vector<uint>bench={}, bool randMap = false);
  ~mutantGen();

  void generate(uint _depth, uint _width, uint _nodes,
                uint _eDensity, uint _eLength,
                uint _nbCluster, uint _maxIter,
                std::vector<uint>bench, bool randMap);

  void save(std::string file);
  void load(std::string file);

  /// Getter
  uint getDepth(){return depth;};
  uint getWidth(){return width;};
  uint getEdgesLen(){return edgesLength;};

  nodes** getNodes(){ return mtx_nodes;};
  nodes* getNode(uint x_pos, uint y_pos){ return mtx_nodes[_2d_node(x_pos, y_pos)];};

  uint* getEdges(){ return mtx_edges;};
  uint*  getInEdge(uint x_start, uint y_start, uint x_trgt, uint y_trgt)
  { return &mtx_edges[_inEdges(x_start, y_start, x_trgt, y_trgt)];};
  uint*  getOutEdge(uint x_start, uint y_start, uint x_trgt, uint y_trgt)
  { return &mtx_edges[_outEdges(x_start, y_start, x_trgt, y_trgt)];};

  /// Debug
  void rawDisplay();
};

#endif /*_MUTANTGEN*/
