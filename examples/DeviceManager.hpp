#pragma once
#include <string>
#include <vector>
#include "DeviceThread.hpp"


struct DeviceManager
{
    bool add_device(const std::string &port_path) {
        // detection
        auto device_thread = std::make_unique<DeviceThread>(port_path);
        {
            std::thread thread_unit(&DeviceThread::operator(), device_thread.get());
            thread_unit.detach();
        }

        auto status = device_thread->device_detected();
        if (status != DeviceThread::DeviceDetected::detected)
            return false;

        {
            std::lock_guard<std::mutex> ml(_access_mutex);
            const auto device_name = device_thread->device_name();
            _connected_devices.emplace_back(std::make_pair(device_name, std::move(device_thread)));
        }

        return true;
    }

    void remove_disconnected_devices() {
        for (size_t i = 0; i < _connected_devices.size(); ++i)
            if (!_connected_devices[i].second->is_connected())
                _connected_devices.erase(_connected_devices.begin() + i);
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