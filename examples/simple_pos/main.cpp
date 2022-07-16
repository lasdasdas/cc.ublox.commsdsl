#include <iostream>
#include <stdexcept>
#include <csignal>

#include "Session.h"
#include "ProgramOptions.h"

struct Session2 : ublox::simple_pos::Session {
  Session2(ublox::common::boost_wrap::io& io, const std::string& dev, unsigned int baudrate)
    : Session( io,  dev,  baudrate){

  }

    //void handle(InNavPosllh& msg){(void)msg; std::cout << "got1" << std::endl;}
    void handle(InNavAttMsg& msg){(void)msg; /*std::cout << "got2" << std::endl; */}
    ////void handle(InEsfStatus& msg){(void)msg; std::cout << "got3" << std::endl;};
    void handle(InEsfAlg& msg){(void)msg; std::cout << "got4" << std::endl;};
    //void handle(InNavTimeutc& msg){(void)msg; std::cout << "got5" << std::endl;};;
    void handle(InNavPvt& msg){(void)msg;  std::cout << 8 << std::endl; };;
    //void handle(InNavSol& msg){(void)msg; std::cout << "got6" << std::endl;};

};

int main(int argc, const char* argv[])
{
    try {
        ublox::simple_pos::ProgramOptions options;
        options.parse(argc, argv);
        if (options.helpRequested()) {
            std::cout << "Usage:\n\t" << argv[0] << " [OPTIONS]\n";
            options.printHelp(std::cout);
            return 0;
        }

        ublox::common::boost_wrap::io io;

        boost::asio::signal_set signals(io, SIGINT, SIGTERM);
        signals.async_wait(
            [&io](const boost::system::error_code& ec, int signum)
            {
                io.stop();
                if (ec) {
                    std::cerr << "ERROR: " << ec.message() << std::endl;
                    return;
                }

                std::cerr << "Termination due to signal " << signum << std::endl;
            });

        Session2 session(io, options.device(), 921600);
        if (!session.start()) {
            return -1;
        }

        io.run();
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: Unexpected exception: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
