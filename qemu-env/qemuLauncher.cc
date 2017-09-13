/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

#include <iostream>
#include "utility/qemuBuilder.h"

using namespace std;

int main(int argc, char* argv[])
{
  if (1 == argc){
    std::cerr << "Error: Pass configuration filename as arguments. \n";
    return EXIT_FAILURE;
  }
    qb::qemuBuilder *qemu_starter;

    //Open configFile
    ifstream  configFile;
    configFile.open(argv[1]);
    qemu_starter = new qb::qemuBuilder(configFile, 'j');
    configFile.close();
    return EXIT_SUCCESS;
}
