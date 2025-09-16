#include <iostream>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyflie.h>
#include "logger.hpp"

int main(int argc, char **argv)
{

  std::string uri;
  std::string defaultUri("radio://0/80/2M/E7E7E7E7E7");
  bool verbose = false;

  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("uri", po::value<std::string>(&uri)->default_value(defaultUri), "unique ressource identifier")
    ("verbose,v", "verbose output")
  ;

  try
  {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 0;
    }
    verbose = vm.count("verbose");
  }
  catch(po::error& e)
  {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  try
  {
    CrazyflieToolsLogger logger(verbose);
    Crazyflie cf(uri, logger);
    cf.requestMemoryToc();

     std::for_each(cf.memoriesBegin(), cf.memoriesEnd(),
      [](const Crazyflie::MemoryTocEntry& entry)
      {
        std::cout << (int)entry.id << ": " << std::endl;
        std::cout << "  type: ";
        switch (entry.type) {
        case Crazyflie::MemoryTypeEEPROM:
          std::cout << "EEPROM";
          break;
        case Crazyflie::MemoryTypeOneWire:
          std::cout << "1-Wire";
          break;
        case Crazyflie::MemoryTypeLED12:
          std::cout << "LED12";
          break;
        case Crazyflie::MemoryTypeLOCO:
          std::cout << "LOCO";
          break;
        case Crazyflie::MemoryTypeTRAJ:
          std::cout << "TRAJ";
          break;
        case Crazyflie::MemoryTypeLOCO2:
          std::cout << "LOCO2";
          break;
        case Crazyflie::MemoryTypeLH:
          std::cout << "LH";
          break;
        case Crazyflie::MemoryTypeTester:
          std::cout << "Tester";
          break;
        case Crazyflie::MemoryTypeUSD:
          std::cout << "USD";
          break;
        case Crazyflie::MemoryTypeLEDMem:
          std::cout << "LEDMem";
          break;
        case Crazyflie::MemoryTypeApp:
          std::cout << "App";
          break;
        case Crazyflie::MemoryTypeDeckMem:
          std::cout << "DeckMem";
          break;
        default:
          std::cout << "Unknown type (" << (int)entry.type << ")!";
          break;
        }
        std::cout << std::endl;
        std::cout << "  size: " << entry.size << std::endl;
        std::cout << "  addr: " << entry.addr << std::endl;
      }
    );

     // std::vector<uint8_t> data;
         // cf.readMemory(0, data);

         return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
