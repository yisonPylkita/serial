#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <cassert>
#include <functional>
#include <experimental/filesystem>
#include <serial/serial.h>
#include "stl_utils.hpp"
#include "DeviceManager.hpp"
#include "DeviceThread.hpp"


using namespace vec::operators;

DeviceManager device_manager;

void watch_port_dirs()
{
    // device watcher
    bool exit_loop = false;
    std::vector<std::string> ignored_ports;

    while (!exit_loop) {
        // remove disconnected ports
        const auto freed_ports = device_manager.remove_disconnected_devices();
        for (const auto &port : freed_ports)
            ignored_ports = vec::remove_all(ignored_ports, port);

        // list available ports
        // TODO: fix it
        std::vector<std::string> found_ports;
        {
            namespace fs = std::experimental::filesystem;
            for (auto &p : std::experimental::filesystem::directory_iterator(fs::path("/dev")))
                if (p.path().string().substr(0, 11) == "/dev/ttyUSB")
                    found_ports.push_back(p.path().string());

            std::vector<std::string> new_ignored_ports;
            for (auto &&ignored_port : ignored_ports)
                if (vec::contains(found_ports, ignored_port))
                    new_ignored_ports.push_back(ignored_port);
            ignored_ports = new_ignored_ports;
        }

        // check each one of them
        for (const auto &port: found_ports) {
            if (vec::contains(ignored_ports, port))
                continue;

            std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
            start = std::chrono::high_resolution_clock::now(); // TODO: bench
            device_manager.add_device(port); // try register as new device
            ignored_ports += port;
            // TODO: bench
            end = std::chrono::high_resolution_clock::now();
            std::cout << "It took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                      << "ms to detect " << port << " device" << std::endl;
        }
    }
}

int main()
{
    std::thread ports_watch_thread(watch_port_dirs);

    // listen for API requests
    bool exit_app = false;
    while (!exit_app)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ports_watch_thread.join();
    return EXIT_SUCCESS;
}