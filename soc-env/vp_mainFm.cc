/*
 * TLMu System-C TLM-2.0 example app.
 *
 * Copyright (c) 2011 Edgar E. Iglesias.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <inttypes.h>

#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "tlm_utils/tlm_quantumkeeper.h"

using namespace sc_core;
using namespace std;

//include utility
#include "models/utility/socBuilder.h"
//include models
#include "models/structural/iconnect.h"
#include "models/structural/memory.h"
#include "models/structural/amba_GP.h"
//include monitoring
#include "models/monSystem/monSystem.h"

//#define GDB      NULL
//#define GDB      "tcp::1234"

int sc_main(int argc, char* argv[])
{
  if (1 == argc){
    std::cerr << "Error: Pass configuration filename as arguments. \n";
    exit(-1);
  }
    sb::SoCBuilder *vPlatf;

    //Open configFile
    ifstream  configFile;
    configFile.open(argv[1]);
    vPlatf = new sb::SoCBuilder(configFile, 'j');
    configFile.close();

    //Save SoCconfig
    ofstream  outFile;
    outFile.open("socOut.json");
    cereal::exportConfig<sb::socCnf>(vPlatf->getTopLevel(), outFile, 'j');
    outFile.close();

    return 0;
}
