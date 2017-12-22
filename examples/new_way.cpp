#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <cassert>
#include <functional>
#include <serial/serial.h>
#include "stl_utils.hpp"
#include "DeviceManager.hpp"
#include "DeviceThread.hpp"


using namespace vec::operators;

DeviceManager device_manager;

void watch_port_dirs()
{
    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;

    // device watcher
    bool exit_loop = false;
    while (!exit_loop) {
        device_manager.remove_disconnected_devices();
        // TODO: add ignored_devices
        std::vector<std::string> found_ports{"/dev/ttyUSB0"};
        for (size_t i = 0; i < found_ports.size(); ++i) {
            start = std::chrono::high_resolution_clock::now(); // TODO: bench
            const auto port = found_ports[i];
            found_ports.erase(found_ports.begin() + i);
            // detection
            auto device_thread = std::make_unique<DeviceThread>(port);
            {
                std::thread thread_unit(&DeviceThread::operator(), device_thread.get());
                thread_unit.detach();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            auto status = device_thread->device_detected();
            if (status == DeviceThread::DeviceDetected::uninitialized) {
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
                auto new_status = device_thread->device_detected();
                if (new_status == DeviceThread::DeviceDetected::uninitialized ||
                    new_status == DeviceThread::DeviceDetected::not_detected)
                    // give up
                    continue;
                else if (new_status == DeviceThread::DeviceDetected::detected)
                    status = DeviceThread::DeviceDetected::detected;
            }
            if (status != DeviceThread::DeviceDetected::detected)
                continue;

            // register as new device
            device_manager.add_device(std::move(device_thread));
            // TODO: bench
            end = std::chrono::high_resolution_clock::now();
            std::cout << "It took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                      << "ms to start/stop thread" << std::endl;
        }
    }
}

int main()
{
    std::thread ports_watch_thread(watch_port_dirs);
    ports_watch_thread.detach();

    // listen for API requests
    bool exit_app = false;
    while (!exit_app)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return EXIT_SUCCESS;
}