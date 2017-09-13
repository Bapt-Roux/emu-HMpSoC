/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * File: cerealExt.h
 * declaration of templated class for serialization
 * added it in cereal namespace
 */

#ifndef _CEREAL_EXT
#define _CEREAL_EXT

//Cereal header
#include <cereal/types/map.hpp> //types
#include "cereal/archives/json.hpp" //archives format
#include "cereal/archives/xml.hpp"

//c++ stream
#include <fstream>

namespace cereal{
  /*
   * struct serialMap
   * Change map serialization for cereal
   */
  template<typename blkType> struct serialMap{
    std::map<std::string, blkType> sMap;

    // Offload functions for textual in/out
    template <class Archive,
      cereal::traits::EnableIf<cereal::traits::is_text_archive<Archive>::value> = cereal::traits::sfinae> inline
      void save(Archive & ar) const
    {
      for (const auto & i : sMap)
        ar(cereal::make_nvp(i.first, i.second));
    }
      template <class Archive,
      cereal::traits::EnableIf<cereal::traits::is_text_archive<Archive>::value> = cereal::traits::sfinae> inline
      void load(Archive & ar)
    {
      sMap.clear();
      auto hint = sMap.begin();
      while (true)
        {
          const auto namePtr = ar.getNodeName();
          if (!namePtr)
            break;

          std::string key = namePtr;
          blkType value; ar(value);
          hint = sMap.emplace_hint(hint, std::move(key), std::move(value));
        }
    }
  };

  /*
   * free templated functions for import/export config from stream
   */
  template<typename serialType> void exportConfig(serialType & trgt, std::ofstream &stream, char sType)
    {
      if (stream.is_open()){
        switch (sType){
        case 'j' : //"JSON"
          {
            cereal::JSONOutputArchive oarchive(stream);
            trgt.serialize(oarchive);
            break;
          }
        case 'x' : //"XML"
          {
            cereal::XMLOutputArchive oarchive(stream);
            trgt.serialize(oarchive);
            break;
          }
        default:
          std::cerr << "monSysMaster" << "Unsupported export type (options are j: JSON, x: XML)\n";
          exit(-1);
          break;
        }
      }else{
        std::cerr << "monSysMaster" << " Output stream not open correctly \n";
        exit(-1);
      }
      return;
    }

  template<typename serialType> void importConfig(serialType & trgt, std::ifstream &stream, char sType)
    {
      if (stream.is_open()){
        switch (sType){
        case 'j' : //"JSON"
          {
            cereal::JSONInputArchive iarchive(stream);
            trgt.serialize(iarchive);
          }
          break;

        case 'x' : //"XML"
          {
            cereal::XMLInputArchive iarchive(stream);
            trgt.serialize(iarchive);
          }
          break;
        default:
          std::cerr << "SerialStream" << "Unsupported import type (options are j: JSON, x: XML)\n";
          exit(-1);
          break;
        }
      }else{
        std::cerr << "SerialStream" << " Input stream not open correctly \n";
        exit(-1);
      }
      return;
    }

  /*
   * free templated functions for loading/saving hex value from string format
   * Json doesn't enable hex value natively
   */
  template<class Archive> ulong getHex(Archive &ar){
    std::string strBuffer; // get value as string
    ar(strBuffer);
    if(strBuffer.compare(0,2,"0x")){ // convert to uint
      return std::stoul(strBuffer,NULL,10);
    } else{
      return std::stoul(strBuffer,NULL,16);
    }
  }

  template<class Archive> void saveHex(Archive &ar, std::string name, ulong value){
    std::stringstream streamBuffer; // convert to string
    streamBuffer << std::hex << "0x" << value;
    ar(cereal::make_nvp(name.c_str(),streamBuffer.str()));
    // streamBuffer.str(""); streamBuffer.clear();
  }

}// end namespace cereal
#endif /*_CEREAL_EXT*/
