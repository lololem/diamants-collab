#include <iostream>
#include <crazyflie_cpp/Crazyflie.h>

class CrazyflieToolsLogger
    : public Logger
{
public:
    CrazyflieToolsLogger(bool verbose)
        : verbose_(verbose)
    {
    }

    virtual void info(const std::string & msg)
    {
        if (verbose_) {
            std::cout << "INFO " << msg << std::endl;
        }
    }
    
    virtual void warning(const std::string & msg)
    {
        std::cout << "WARNING " << msg << std::endl;
    }
    
    virtual void error(const std::string & msg) 
    {
        std::cout << "ERROR " << msg << std::endl;
    }

private:
    bool verbose_;
};