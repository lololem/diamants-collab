#include <iostream>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyflie.h>


int main(int argc, char **argv)
{

  std::string addressStr;
  std::string defaultAddressStr("0xE7E7E7E7E7");
  // bool verbose = false;

  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("address", po::value<std::string>(&addressStr)->default_value(defaultAddressStr), "device address")
    // ("verbose,v", "verbose output")
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
    // verbose = vm.count("verbose");
  }
  catch(po::error& e)
  {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  try
  {
    uint64_t address;
    std::stringstream sstr;
    sstr << std::hex << addressStr;
    sstr >> address;

    const auto uris = Crazyflie::scan(address);

    for (const auto &uri : uris) {
      std::cout << uri << std::endl;
    }
    return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
