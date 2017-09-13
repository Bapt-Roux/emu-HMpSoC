/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: nocTest.cc
 * Test application for nocIPC, two available modes:
 *    * traffic producer
 *    * traffic consumer
 */

//NOTE: Don't use memcpy because  it generate Byte access on bus

#include <iostream>
#include <sstream>
#include "noc_helper.h"
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>

#define DEFAULT_PLENGTH 320
/* #define DISABLE_MEMCALL */

//register getter/setter
#define _RNOC(NAME) (((uintptr_t)noc_register+ (RNOC_##NAME)))
#define NOC_STATUS (*((volatile uint8_t*)_RNOC(CMD)))

using namespace std;
using namespace noc;

void usage(char * name)
{
  printf("%s -g <@noc> -n|-d|-o|-s -p|-c [-l payload_length]\n", name);
  printf("\t -g <@noc>    base address of noc \n");
  printf("\t -s    sync packet\n");
  printf("\t -o    order packet\n");
  printf("\t -p    produce packet\n");
  printf("\t -c    consume packet\n");
  printf("\t -d    discover Noc \n");
  printf("\t -n    get Noc powerLog \n");
  printf("\t -l <payload_length>   specify payload length [optional]\n");
  return;
}

long unsigned get_luint(string displayMessage)
{
  string buffer;
  stringstream ss;
  long unsigned value;
  do{
    cout <<displayMessage;
    getline(cin, buffer, '\n');
    if("0x" == buffer.substr(0,2))
      ss << hex<< buffer.substr(2);
    else
      ss << buffer;

    if(ss >> value)
      break;
    else{
      ss.str(""); //purge stream;
      ss.clear();
      cout << "Bad value \n";
    }
  }while(1);
  return value;
}

int main(int argc, char *argv[])
{
  int c;
  unsigned long noc_addr= 0x00;
  char p_type=0;
  char p_mode=0;
  unsigned int p_length= DEFAULT_PLENGTH;

  /* Parse command line arguments */
  while((c = getopt(argc, argv, "dnospcl:h")) != -1) {
    switch(c) {
    case 'o':
      p_type = 'o'; // order packet
      break;
    case 's':
      p_type = 's'; // sync packet
      break;
    case 'p':
      p_mode = 'p'; // producer
      break;
    case 'c':
      p_mode = 'c'; // consumer
      break;
    case 'd':
      p_type = 'd'; // nocDiscover
      p_mode = 'd';
      break;
    case 'n':
      p_type = 'n'; // get nocPower
      p_mode = 'n';
      break;
    case 'l':
      p_length = strtoul(optarg,NULL, 0);
      break;
    case 'h':
      usage(argv[0]);
      return EXIT_SUCCESS;
    default:
      printf("invalid option: %c\n", (char)c);
      usage(argv[0]);
      return EXIT_FAILURE;
    }
  }
  if (p_type == 0) {
    printf("Specify %s packet type : -d \"discoverNoC\" -s \"sync\" -o \"order\".\n", argv[0]);
    return EXIT_FAILURE;
  }
  if (p_mode == 0) {
    printf("Specify %s mode : -p \"producer\" -c \"consumer\".\n",argv[0]);
    return EXIT_FAILURE;
  }
  if (noc_addr == 0) {
    printf("Unset noc address: use the default 0x%lx.\n",DEFAULT_NOC_ADDR);
    noc_addr = DEFAULT_NOC_ADDR;
  }

  /*
   * Map NocItf in process memory
   */
#ifndef DISABLE_MEMCALL
  int noc_fd;
  void* noc_register;
  /* Open /dev/mem file */
  noc_fd = open ("/dev/mem", O_RDWR);
  if (noc_fd < 1) {
    perror(argv[0]);
    return -1;
  }
  noc_register = mmap(NULL,0x70, PROT_READ|PROT_WRITE, MAP_SHARED, noc_fd, noc_addr);
#endif

  stringstream message;

  if ('p' == p_mode){   //producer
    if ('o' == p_type){ //order
      orderHeader order;
      order.is_sync_header= false;
      order.data_length = p_length;
      //TODO allocate targeted memory segment
      /*
       * Ask user for order content
       */
      message.str("");
      message <<"Order Type available: ";
      for(auto it : order_helper){ message << it.first << ": " << it.second <<"; ";};
      message <<"\n choose: ";
      do{
        long unsigned int readValue = get_luint(message.str());
        if((readValue >= order_helper.begin()->first) && (readValue <= prev(order_helper.end())->first)){
          order.cmd= (order_t) readValue;
          break;
        } else
          cout <<"Value entered out of range";
      }while(1);

      message.str("");
      message <<"job_id (task_id MSB)\n";
      order.task_id= get_luint(message.str())<<16;

      message.str("");
      message <<"order_id (task_id LSB)\n";
      order.task_id += get_luint(message.str())&0xFFFF;

      message.str("");
      message <<"remote address \n";
      order.remote= get_luint(message.str());

      if((NOC_MEMGET == order.cmd)||(NOC_MEMRTV ==order.cmd)){
        message.str("");
        message <<"local address \n";
        order.local= get_luint(message.str());
      }else if(NOC_ACK == order.cmd){
        message.str("");
        message <<"Ack Type available: ";
        for(auto it : ack_helper){ message<< it.first <<": " << it.second <<"; ";};
        message <<"\n choose: ";
        do{
          long unsigned int readValue = get_luint(message.str());
          if((readValue >= ack_helper.begin()->first) && (readValue <= prev(ack_helper.end())->first)){
            order.local= (ack_t) readValue;
            break;
          } else
            cout <<"Value entered out of range";
        }while(1);
      }
      /*Send data on noc */
#ifndef DISABLE_MEMCALL
      while(0x00 != NOC_STATUS); // wait Noc rdy
      *((orderHeader*)_RNOC(RCVDATA)) = order; //set data request
      NOC_STATUS = CNOC_SORDER; //send cmd
#endif
      cout << "Order sent: \n";
      cout << "\t task_id: "<< order.task_id<<" \n";
      string rdCmd = order_helper.find(order.cmd)->second;
      cout << "\t cmd: "<<dec<< rdCmd<<" \n";
      cout << "\t @local: 0x"<<hex<< order.local<<" \n";
      cout << "\t @remote: 0x"<<hex<< order.remote<<" \n";
      cout << "\t @data: "<<dec<<order.data_length<<" \n";

    } else{             //sync
      waitPoint sync;
      waitStatus wakeState;
      /*
       * Ask user for waitPoint content
       */
      message.str("");
      message <<"job_id (task_id MSB)\n";
      sync.task_id = get_luint(message.str())<<16;

      message.str("");
      message <<"order_id (task_id LSB)\n";
      sync.task_id += get_luint(message.str())&0xFFFF;

      message.str("");
      message <<"Ack Type available: ";
      for(auto it : ack_helper){ message<< it.first <<": " << it.second <<"; ";};
      message <<"\n choose: ";
      do{
        long unsigned int readValue = get_luint(message.str());
        if((readValue >= ack_helper.begin()->first) && (readValue <= prev(ack_helper.end())->first)){
          sync.ack_type= (ack_t) readValue;
          break;
        } else
          cout <<"Value entered out of range";
      }while(1);
      /*Send data on noc */
#ifndef DISABLE_MEMCALL
      while(0x00 != NOC_STATUS); //wait Noc rdy
      *((waitPoint*)_RNOC(RCVDATA)) = sync;
      NOC_STATUS = CNOC_SYNC; //send cmd
      while(0x00 != NOC_STATUS); // wait Noc rdy
      wakeState = *((waitStatus*)_RNOC(SENDDATA));
#endif
      if ( WAITPOINT_SET == wakeState){
        cout << "WaitPoint set valid: \n";
        cout << "\t task_id: 0x"<<hex<< sync.task_id<<" \n";
        string rdAck = ack_helper.find(sync.ack_type)->second;
        cout << "\t ack_type: 0x"<<hex<< rdAck<<" \n";
      }else{
        cout << "Invalid WaitPoint: event already occur or irrelevant event\n"
          << "\t => Instant WakeUp of the asking task \n";
      }
    }
  } else if ('c' == p_mode){               //consumer
    if ('o' == p_type){ //order
      orderHeader order;
      order.is_sync_header=false;
      /*Send data on noc */
#ifndef DISABLE_MEMCALL
      while(0x00 != NOC_STATUS); // wait Noc rdy
      NOC_STATUS = CNOC_GORDER; //send cmd
      while(0x00 != NOC_STATUS); // wait Noc rdy
      order = *((orderHeader*)_RNOC(SENDDATA));
#endif
      if(order.is_sync_header)
        cout << "Order vector is empty \n";
      else{
        cout << "Received order: \n";
        cout << "\t task_id: 0x"<< hex<< order.task_id<<" \n";
        string rdCmd = order_helper.find(order.cmd)->second;
        cout << "\t cmd: "<<dec<< rdCmd<<" \n";
        cout << "\t @local: 0x"<<hex<< order.local<<" \n";
        cout << "\t @remote: 0x"<<hex<< order.remote<<" \n";
        cout << "\t @data: "<<dec<< order.data_length<<" \n";
      }
    } else{             //sync
      waitPoint wakeup;
      /*Send data on noc */
#ifndef DISABLE_MEMCALL
      while(0x00 != NOC_STATUS); // wait Noc rdy
      NOC_STATUS = CNOC_WAKEUP; //send cmd
      while(0x00 != NOC_STATUS); // wait Noc rdy
      wakeup = *((waitPoint*)_RNOC(SENDDATA));
#endif
      if(0 == wakeup.task_id)
        cout << "wakeup vector is empty \n";
      else{
        cout<< "WakeUp task: 0x"<<hex<< wakeup.task_id
            <<"("<< ack_helper.find(wakeup.ack_type)->second << ") \n";
      }
    }
  }else if ('d' == p_mode){   // NoC discovery
    uint64_t clNb=0;
    /*Send data on noc */
#ifndef DISABLE_MEMCALL
    while(0x00 != NOC_STATUS); // wait Noc rdy
    NOC_STATUS = CNOC_DISCOVER; //send cmd
    while(0x00 != NOC_STATUS); // wait Noc rdy
    clNb = *((uint64_t*)_RNOC(SENDDATA));
    uint64_t buffer[clNb+1];
    memcpy(buffer,(void*)_RNOC(SENDDATA),(clNb)*sizeof(uint64_t));
#endif
    cout << "Cluster available are ["<<clNb<<"]:\n";
    for( uint i=1; i<=clNb; i++){
      cout << buffer[i] << "\t @0x"<<hex<<(buffer[i] >>32)<<": ["
        <<((buffer[i]&0xFF00)>>8) <<", "<<(buffer[i]&0x00FF)<<"]\n";
    }
  }else if ('n' == p_mode){   // get NoC powerLog
    uint64_t buffer;
    uint32_t extractEnj;
    /*Send data on noc */
#ifndef DISABLE_MEMCALL
    while(0x00 != NOC_STATUS); // wait Noc rdy
    NOC_STATUS = CNOC_NOCPOWER; //send cmd
    while(0x00 != NOC_STATUS); // wait Noc rdy
    buffer = *((uint64_t*)_RNOC(SENDDATA));
#endif
    extractEnj = ((buffer>>32) &0xFFFFFFFF);
    cout << "Noc powerLog value are:\n"
      <<"\t Energy[J]: "<< (float) *(reinterpret_cast<float*>(&extractEnj)) <<"\n"
      <<"\t time[ns]: "<<dec<< (uint32_t) (buffer &0xFFFFFFFF)<<"\n";
  }

#ifndef DISABLE_MEMCALL
  /* Unmap memory segement and close file descriptor*/
  munmap(noc_register, 0x70);
  close(noc_fd);
#endif


  return 0;
}
