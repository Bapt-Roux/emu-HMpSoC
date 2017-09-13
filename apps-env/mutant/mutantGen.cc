/**
 * @file mutantGen.cc
 * @brief application mutant generator functions
 */
#include "mutantGen.h"

/**
 * @brief initialize memory for nodes and edges matrix
 * @return void
 */
void mutantGen::memorySetup()
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
 * @brief generate a random mutant structure following class properties
 * @return void
 */
void mutantGen::randomGen(std::vector<uint>bench, bool randMap)
{
  srand(time(NULL));
  int nodePool = nbNodes;
  print_DMGEN("start randomGen with: "<< "nbNodes " << nbNodes <<"\n"
              << "depth " << depth << ", width " << depth <<"\n"
              << "Edges density " << edgesDensity << ", length " << edgesLength<< "\n"
              << "Number of clusters " << nbCluster << ", maximum iterations " << maxIter <<"\n");

  for (uint r=0; r < depth; r++){
    for( uint c=0; c < width; c++){
      //Clean entry
      if (NULL != mtx_nodes[_2d_node(c, r)]){delete mtx_nodes[_2d_node(c, r)];}

      uint randVal = random();
      uint comIn =0;
      for(uint i=0; i < (depth*width); i++){
        comIn += mtx_edges[_inEdges(c, r, i, 0)];
      }
      if((0 < comIn) // node already connected or randCreated
         || ((0==r) && (0==c)) //firstNode
         ||((0< nodePool) && (randVal&0x1))){
        print_DMGEN(" Node(d,w) ["<< r <<", "<< c << "] created \n");
        nodePool--;
        mtx_nodes[_2d_node(c, r)] = new nodes;
        mtx_nodes[_2d_node(c, r)]->id = nbNodes - nodePool;
        mtx_nodes[_2d_node(c, r)]->task_id = bench.at(((randVal >> 24)&0xff)%bench.size());
        mtx_nodes[_2d_node(c, r)]->nbIteration = (((randVal >> 16)&0xff)% maxIter)+1;
        if(randMap){
          mtx_nodes[_2d_node(c, r)]->hwTask = ((randVal >> 1)&0x1);
          mtx_nodes[_2d_node(c, r)]->clTarget = ((randVal >> 2)&0xff)%nbCluster;
        }else{
          mtx_nodes[_2d_node(c, r)]->hwTask = false;
          mtx_nodes[_2d_node(c, r)]->clTarget = 0xff;
        }
        uint edgesPool = (((randVal )&0xffff)% edgesDensity)+1;
        while( (0 < edgesPool) && ((depth-1) >r)){
          uint edgeRand = random();
          uint colJump = ((edgeRand&0xffff)%width);
          uint lvlJump = (((edgeRand>> 16)&0xffff)%edgesLength)+1;
          if(depth <= (r+lvlJump)){//limite communication to last lvl
            lvlJump = (depth - r -1);
          }
          edgesPool--;
          mtx_edges[_outEdges(c,r, colJump, r+lvlJump)] = 1;
          print_DMGEN("edge added between [" <<r << ", "<< c << "] and ["
                      << r + lvlJump<< ", "<< colJump << "] \n");
        }
      }
    }
  }

  rawDisplay();
}

/**
 * @brief generated mutantGen constructor
 * @param depth: max graph depth
 * @param width: max graph width
 * @param nbNodes: number of nodes in graph
 */
mutantGen::mutantGen(uint _depth, uint _width, uint _nodes, uint _eDensity, uint _eLength, uint _nbCluster, uint _maxIter, std::vector<uint>bench, bool randMap)
  : depth(_depth), width(_width), nbNodes(_nodes),
    edgesDensity(_eDensity), edgesLength(_eLength),
    nbCluster(_nbCluster), maxIter(_maxIter),
    mtx_nodes(NULL), mtx_edges(NULL)
{
  memorySetup();
  randomGen(bench, randMap);
}

/**
 * @brief mutantGen constructor from filepath
 * @param filepath: path to graph filename
 */
mutantGen::mutantGen(std::string filepath)
{
  load(filepath);
}

/**
 * @brief mutantGen destructor
 */
mutantGen::~mutantGen()
{
  if (NULL == mtx_nodes){
    delete[] mtx_nodes;
  }

  if (NULL == mtx_edges){
    delete[] mtx_edges;
  }
}

/**
 * @brief generate/reGenerate mutantGen with given properties
 * @param depth: max graph depth
 * @param width: max graph width
 * @param nbNodes: number of nodes in graph
 * @param eDensity: max number of edges from one node
 * @param eLength: max level cross by edges
 * @return void
 */
void mutantGen::generate(uint _depth, uint _width, uint _nodes, uint _eDensity, uint _eLength, uint _nbCluster, uint _maxIter, std::vector<uint>bench, bool randMap)
{
  // Setup new graph parameters
  depth = _depth;
  width = _width;
  nbNodes = _nodes;
  edgesDensity = _eDensity;
  edgesLength = _eLength;
  nbCluster = _nbCluster;
  maxIter = _maxIter;

  // generate a new graph
  memorySetup();
  randomGen(bench, randMap);

  return;
}

/**
 * @brief Save to file in binary form
 * @param file: file path
 */
void mutantGen::save(std::string file){
  std::ofstream outFile;
  const bool empty = false;
  const bool full = true;
  //open file
  outFile.open(file.c_str(), std::ios::out | std::ios::binary);
  //dump general properties
  outFile.write((char*)&depth, (int)sizeof(uint));
  outFile.write((char*)&width, (int)sizeof(uint));
  outFile.write((char*)&nbNodes, (int)sizeof(uint));
  outFile.write((char*)&edgesDensity, (int)sizeof(uint));
  outFile.write((char*)&edgesLength, (int)sizeof(uint));
  outFile.write((char*)&nbCluster, (int)sizeof(uint));
  outFile.write((char*)&maxIter, (int)sizeof(uint));

  //dump nodes existance and values
  for( uint n=0; n< (depth*width); n++){
    if(NULL == mtx_nodes[n]){
      outFile.write((char*)&empty, (int)sizeof(bool));
    }else{
      outFile.write((char*)&full, (int)sizeof(bool));
      // Write node value
      outFile.write((char*)(&mtx_nodes[n]->task_id), (int)sizeof(uint));
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
void mutantGen::load(std::string file){
  std::ifstream inFile;
  //open file
  inFile.open(file.c_str(), std::ios::in | std::ios::binary);

  //read general properties
  inFile.read((char*)&depth, (int)(int)sizeof(uint));
  inFile.read((char*)&width, (int)sizeof(uint));
  inFile.read((char*)&nbNodes, (int)sizeof(uint));
  inFile.read((char*)&edgesDensity, (int)sizeof(uint));
  inFile.read((char*)&edgesLength, (int)sizeof(uint));
  inFile.read((char*)&nbCluster, (int)sizeof(uint));
  inFile.read((char*)&maxIter, (int)sizeof(uint));

  memorySetup();

  //allocate memory and read nodes values
  bool exist;
  for( uint n=0; n< (depth*width); n++){
    inFile.read((char*)&exist, (int)sizeof(bool));
    if(0 != exist){//read values
      mtx_nodes[n] = new nodes;
      inFile.read((char*)(&mtx_nodes[n]->task_id), (int)sizeof(uint));
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
 * @note enable MGEN_DEBUG
 */
void mutantGen::rawDisplay(){
#ifdef MGEN_DEBUG
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
