#pragma once
#include <string>
#include <vector>
#include "DeviceThread.hpp"


struct DeviceManager
{
    void add_device(DeviceThread::Instance instance) {
        std::lock_guard<std::mutex> ml(_access_mutex);
        const auto device_name = instance->device_name();
        _connected_devices.emplace_back(std::make_pair(device_name, std::move(instance)));
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