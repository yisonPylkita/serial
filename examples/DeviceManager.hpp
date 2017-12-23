#pragma once
#include <string>
#include <vector>
#include <thread>
#include "DeviceThread.hpp"


struct DeviceManager
{
    void add_device(const std::string &port_path) {
        // detection
        auto device_thread = std::make_unique<devices::Fm30>(port_path);
        {
            std::thread thread_unit(&devices::Fm30::device_loop, device_thread.get());
            thread_unit.detach();
        }

        auto status = device_thread->device_detected();
        if (status != DeviceThread::DeviceDetected::detected)
            return;

        {
            std::lock_guard<std::mutex> ml(_access_mutex);
            const auto device_name = device_thread->device_name();
            _connected_devices.emplace_back(std::make_pair(device_name, std::move(device_thread)));
        }
    }

    std::vector<std::string> remove_disconnected_devices() {
        std::vector<std::string> removed_ports;
        for (size_t i = 0; i < _connected_devices.size(); ++i) {
            auto device = _connected_devices[i].second.get();
            if (!device->is_connected()) {
                removed_ports.emplace_back(device->get_port());
                _connected_devices.erase(_connected_devices.begin() + i);
            }
        }

        return removed_ports;
    }

    DeviceThread * get_device_by_name(const std::string &device_name) {
        for (const auto &device : _connected_devices)
            if (device.first == device_name)
                return device.second.get();
    }

private:
    std::mutex _access_mutex;
    std::vector<std::pair<std::string, DeviceThread::Instance>> _connected_devices;
};