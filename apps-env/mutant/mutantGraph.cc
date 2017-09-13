/**
 * @file mutantGraph.cc
 * @brief application mutant grapherator functions
 */
#include "mutantGraph.h"

/**
 * @brief initialize memory for nodes and edges matrix
 * @return void
 */
void mutantGraph::memorySetup()
{
  if (NULL == mtx_nodes){
    delete[] mtx_nodes;
  }
  mtx_nodes = new nodes*[depth*width];
  for(uint i=0; i<(depth*width); i++){mtx_nodes[i] = NULL;}

  if (NULL == mtx_edges){
    delete[] mtx_edges;
  }
  mtx_edges = new uint[(depth*width)*(depth*width)];
  memset(mtx_edges, 0, sizeof(uint)*(depth*width)*(depth*width));

  return;
}


/**
 * @brief mutantGraph constructor from parameters
 * @param depth: max graph depth
 * @param width: max graph width
 * @param nbNodes: number of nodes in graph
 */
mutantGraph::mutantGraph(uint _depth, uint _width, uint _nodes, uint _eDensity, uint _eLength)
  : depth(_depth), width(_width), nbNodes(_nodes),
    edgesDensity(_eDensity), edgesLength(_eLength),
    mtx_nodes(NULL), mtx_edges(NULL)
{
  memorySetup();
}

/**
 * @brief mutantGraph constructor from previous saved graphfile
 * @param graphfile: binary dump of graph properties in file
 */
mutantGraph::mutantGraph(std::string file){
  load(file);
}

/**
 * @brief mutantGraph destructor
 */
mutantGraph::~mutantGraph()
{
  if (NULL == mtx_nodes){
    delete[] mtx_nodes;
  }

  if (NULL == mtx_edges){
    delete[] mtx_edges;
  }
}

/**
 * @brief Save to file in binary form
 * @param file: file path
 */
void mutantGraph::save(std::string file){
  std::ofstream outFile;
  const char empty = 0;
  char nameLen;
  //open file
  outFile.open(file.c_str(), std::ios::out | std::ios::binary);
  //dump grapheral properties
  outFile.write((char*)&depth, (int)sizeof(uint));
  outFile.write((char*)&width, (int)sizeof(uint));
  outFile.write((char*)&nbNodes, (int)sizeof(uint));
  outFile.write((char*)&edgesDensity, (int)sizeof(uint));
  outFile.write((char*)&edgesLength, (int)sizeof(uint));

  //dump nodes existance and values
  for( uint n=0; n< (depth*width); n++){
    if(NULL == mtx_nodes[n]){
      outFile.write((char*)&empty, (int)sizeof(char));
    }else{
      // Write name
      nameLen = mtx_nodes[n]->task_name.length();
      outFile.write((char*)&nameLen, (int)sizeof(char));
      char buffer[nameLen];
      strncpy(buffer, mtx_nodes[n]->task_name.c_str(), nameLen);
      outFile.write((char*)buffer, (int)(nameLen*sizeof(char)));
      // Write others values
      outFile.write((char*)(&mtx_nodes[n]->nbIteration), (int)sizeof(uint8_t));
      outFile.write((char*)(&mtx_nodes[n]->id), (int)sizeof(uint8_t));
      outFile.write((char*)(&mtx_nodes[n]->clTarget), (int)sizeof(uint8_t));
      outFile.write((char*)(&mtx_nodes[n]->hwTask), (int)sizeof(bool));
    }
  }

  //dump edges values
    outFile.write((char*)mtx_edges, (int)((depth*depth)*(width*width)*sizeof(uint)));
  outFile.close();
}

/**
 * @brief Load from file in binary form
 * @param file: file path
 */
void mutantGraph::load(std::string file){
  std::ifstream inFile;
  //open file
  inFile.open(file.c_str(), std::ios::in | std::ios::binary);

  //read grapheral properties
  inFile.read((char*)&depth, (int)(int)sizeof(uint));
  inFile.read((char*)&width, (int)sizeof(uint));
  inFile.read((char*)&nbNodes, (int)sizeof(uint));
  inFile.read((char*)&edgesDensity, (int)sizeof(uint));
  inFile.read((char*)&edgesLength, (int)sizeof(uint));

  memorySetup();

  //allocate memory and read nodes values
  char nameLen;
  for( uint n=0; n< (depth*width); n++){
    inFile.read((char*)&nameLen, (int)sizeof(bool));
    if(0 != nameLen){
      mtx_nodes[n] = new nodes;
      //read bench name
      char buffer[255];
      inFile.read((char*)buffer, (int)(nameLen*sizeof(char)));
      buffer[nameLen] = '\0';
      mtx_nodes[n]->task_name = std::string(buffer);
      //read others values
      inFile.read((char*)(&mtx_nodes[n]->nbIteration), (int)sizeof(uint8_t));
      inFile.read((char*)(&mtx_nodes[n]->id), (int)sizeof(uint8_t));
      inFile.read((char*)(&mtx_nodes[n]->clTarget), (int)sizeof(uint8_t));
      inFile.read((char*)(&mtx_nodes[n]->hwTask), (int)sizeof(bool));
    } else {
      mtx_nodes[n] = NULL;
    }
  }

  //read edges values
  inFile.read((char*) mtx_edges, (int)((depth*depth)*(width*width)*sizeof(uint)));
  inFile.close();

  rawDisplay();
}

/**
 * @brief Display graph in text mode
 * @note enable MGRAPH_DEBUG
 */
void mutantGraph::rawDisplay(){
#ifdef MGRAPH_DEBUG
  std::cout << "matrix edges: \n";
  for(uint mD=0; mD<depth; mD++){
    std::cout <<std::string((depth*3*width), '-') << "\n";
    for(uint mW=0; mW<width; mW++){
      std::cout<<"| ";
      for(uint sD=0; sD<depth; sD++){
        for(uint sW=0; sW<width; sW++){
          if((mD == sD) && (mW == sW)){ //diag
            assert(0 == mtx_edges[((width*mD)+mW)*(depth*width) + ((width*sD)+sW)],
                   "Error node have local edge");
            std::cout << "X";
          }else{
            std::cout << mtx_edges[((width*mD)+mW)*(depth*width) + ((width*sD)+sW)];
          }
          if((width-1) == sW){ //end of subSlot
            std::cout<<"| ";
          }else{
            std::cout<<", ";
          }
        }
      }
      std::cout <<"\n";
    }
  }
  std::cout <<std::string((depth*3*width), '-') << "\n";
#endif
}

/**
 * @brief update node properties
 * @param props: new node properties
 * @param x_pos: x position of the target node
 * @param y_pos: y position of the target node
 */
void mutantGraph::updtNode(nodes props, uint x_pos, uint y_pos){
  nodes *trgt;
  trgt = mtx_nodes[_2d_nodes(x_pos, y_pos)];
  *trgt = props;

  return;
}

/**
 * @brief update node ID property
 * @param id: new node ID
 * @param x_pos: x position of the target node
 * @param y_pos: y position of the target node
 */
void mutantGraph::updtNodeId(uint id, uint x_pos, uint y_pos){
  nodes *trgt;
  trgt = mtx_nodes[_2d_nodes(x_pos, y_pos)];
  trgt->id = id;
  return;
}

/**
 * @brief set edge properties
 * @param val: new edge value
 * @param x_start: x position of the starting node
 * @param y_start: y position of the starting node
 * @param x_dest: x position of the target node
 * @param y_dest: y position of the target node
 */
void mutantGraph::updtEdge(uint val, uint x_start, uint y_start, uint x_dest, uint y_dest){
  mtx_edges[_2d_getEdges(x_start, y_start, x_dest, y_dest)] = val;
  return
}
