/**
 * @file codeGen.h
 * @brief Header file for code generator from mutant application graph
 */
#ifndef _CODE_GEN
#define _CODE_GEN

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>

class codeGen{
private:
  unsigned int indentLvl=0;
  std::ofstream outFile;
  void init_MasterMgmt();

public:
  codeGen(std::string file);
  ~codeGen();
  void genTaskCode(uint baseClId, uint baseTaskId, bool hw, uint iter);


};


#endif /*_CODE_GEN*/
