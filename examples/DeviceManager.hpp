#pragma once
#include <string>
#include <vector>
#include <thread>
#include "DeviceThread.hpp"


struct DeviceInit {
    typedef std::function<DeviceThread::Instance(const std::string &)> create_instance_t;
    typedef std::function<void(DeviceThread *)> device_loop_t;

    DeviceInit(create_instance_t create_instance, device_loop_t device_loop)
    {
        create_instance = create_instance;
        device_loop = device_loop;
    }

    create_instance_t create_instance;
    device_loop_t device_loop;
};

struct DeviceManager
{
    void add_supported_device(DeviceInit device_init)
    {
        _supported_devices.push_back(device_init);
    }

    void add_device(const std::string &port) {
        // detection
        for (auto &&device : _supported_devices) {
            auto device_thread = device.create_instance(port);
            {
                std::thread thread_unit(&device.device_loop, device_thread.get());
                thread_unit.detach();

                auto status = device_thread->device_detected();
                if (status != DeviceThread::DeviceDetected::detected)
                    continue;

                {
                    std::lock_guard<std::mutex> ml(_access_mutex);
                    const auto device_name = device_thread->device_name();
                    _connected_devices.emplace_back(std::make_pair(device_name, std::move(device_thread)));
                }

                break;
            }
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
    std::vector<DeviceInit> _supported_devices;
};