/**
 * @file codeGen.cc
 * @brief code generator from mutant application graph
 */
#include "codeGen.h"

/**
 * @brief codeGen constructor
 */
codeGen::codeGen(std::string file){
  //open target output file
  outFile.open(file.c_str(), std::ios::out);

  //dump comments
  outFile <<"/**\n"
          <<"* @file "<<file.c_str()<<"\n"
          <<"* @brief Application automatically generated from graph structure. Do not edit\n"
          <<"*/\n\n";

  //dump include
  outFile << "#include <>\n";
  outFile << "#include \"\"\n ";
  outFile << "\n\n\n";

  //dump main prototype opening
  outFile << "int main(int argc, char** argv){\n";
  indentLvl++;

  init_MasterMgmt();
}


/**
 * @brief codeGen destructor
 */
codeGen::~codeGen(){
  //dump main prototype closing
  outFile << "return EXIT_SUCCESS\n}\n";
  outFile.close();
}

/**
 * @brief instanciate code for clMaster initialisation
 */
void codeGen::init_MasterMgmt(){
  outFile<<std::string(indentLvl,'\t')
         <<"clMaster clMgmt(\"/dev/sc_noc\");\n";

  //Debug facilities
  outFile<<"#ifdef DEBUG\n";
  outFile<<std::string(indentLvl,'\t')
         <<"const std::map<uint,noc::clPos> confCl = clMgmt.getNocApi().getClusters();\n";
  outFile<<std::string(indentLvl,'\t')
         <<"std::cout << \"NoC configuration read from Master cluster is:\\n\";\n";
  outFile<<std::string(indentLvl,'\t')
         <<"for( auto & it_NoC: confCl){\n";
  outFile<<std::string(indentLvl+1,'\t')
         <<"std::cout << \"\t @\" <<std::hex<< it_NoC.first\n";
  outFile<<std::string(indentLvl+2,'\t')
         <<"<< \" =>[\"<<(int)it_NoC.second.x_pos<< \",\"<<(int)it_NoC.second.y_pos << \"]\\n\";\n";
  outFile<<std::string(indentLvl,'\t')
         <<"}\n";
  outFile<<"#endif\n";
}

void codeGen::genNewBlock(){
  outFile<<std::string(indentLvl,'\t')
         <<"{/n";
  indentLvl++;
}

void codeGen::genEndBlock(){
  indentLvl--;
  outFile<<std::string(indentLvl,'\t')
         <<"}/n";
}

void codeGen::genProviderAndReceiverQueue(){
  outFile<<std::string(indentLvl,'\t')
         <<"std::deque<comProps> providers, receivers;/n";
  outFile<<std::string(indentLvl,'\t')
         <<"comProps tmpComProps;/n";
}

void codeGen::genAddProvider(uint clId, uint taskId, size_t dataLen){
  outFile<<std::string(indentLvl,'\t')
         <<"tmpComProps.clAddr = clId;/n";
  outFile<<std::string(indentLvl,'\t')
         <<"tmpComProps.task = taskId;/n";
  outFile<<std::string(indentLvl,'\t')
         <<"tmpComProps.dSize = dataLen;/n";
  outFile<<std::string(indentLvl,'\t')
         <<"providers.push_back(tmpComProps);/n";
}

void codeGen::genAddReceiver(uint clId, uint taskId, size_t dataLen){
  outFile<<std::string(indentLvl,'\t')
         <<"tmpComProps.clAddr = clId;/n";
  outFile<<std::string(indentLvl,'\t')
         <<"tmpComProps.task = taskId;/n";
  outFile<<std::string(indentLvl,'\t')
         <<"tmpComProps.dSize = dataLen;/n";
  outFile<<std::string(indentLvl,'\t')
         <<"receivers.push_back(tmpComProps);/n";
}

void codeGen::genTaskCode(uint baseClId, uint genTaskId, bool hw, uint iter){
  outFile<<std::string(indentLvl,'\t')
         <<"/n";

  outFile<<std::string(indentLvl,'\t')
         <<"trgtId_t trgt = std::make_pair(task_id, (clId<<28));/n";
    outFile<<std::string(indentLvl,'\t')
           <<"clMgmt.askSlaveSubscribe(trgt);/n";
  outFile<<std::string(indentLvl,'\t')
         <<"clMgmt.setProviderAndReceiver(trgt, providers, receivers);/n";

}


int main(void){
  codeGen test("test.cc");
  test.genTaskCode( 1, 2, 1,5);

  return EXIT_SUCCESS;
}
