#include <iostream>
#include <chrono>
#include <thread>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyflie.h>
#include "logger.hpp"

void onConsoleData(const char* msg)
{
  std::cout << msg;
}

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
    Crazyflie cf(uri, logger, onConsoleData);
    // cf.logReset();
    // cf.requestLogToc();

    // std::unique_ptr<LogBlock<struct log> > logBlock;
    // std::function<void(uint32_t, struct log*)> cb = std::bind(&onLogData, std::placeholders::_1, std::placeholders::_2);

    // logBlock.reset(new LogBlock<struct log>(
    //   &cf,{
    //     {"acc", "x"},
    //     {"acc", "y"},
    //     {"acc", "z"},
    //     {"gyro", "x"},
    //     {"gyro", "y"},
    //     {"gyro", "z"}
    //   }, cb));

    // std::cout << "t,accx,accy,accz,gyrox,gyroy,gyroz" << std::endl;

    // logBlock->start(1); // 10ms

    while (true) {
      cf.sendPing();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
