#pragma once
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <serial/serial.h>


struct DeviceThread
{
    using Instance = std::unique_ptr<DeviceThread>;

    enum class DeviceDetected {
        uninitialized,
        not_detected,
        detected
    };

    explicit DeviceThread(const std::string &port) {
        _port = port;
    }

//    DeviceThread(const DeviceThread &) = delete;
//    void operator=(const DeviceThread &) = delete;

    void operator()() {
        // open port [DevImpl]
        _device_handle = open_port(_port);
        if (!_device_handle) {
            set_device_detected(DeviceDetected::not_detected);
            return;
        }

        // check port [DevImpl]
        if (!check_port(_device_handle.get())) {
            set_device_detected(DeviceDetected::not_detected);
            return;
        }

        // set device name
        // [DevImpl]
        // set_device_name(DevImpl::device_name_impl);
        set_device_name("fm30");
        set_device_detected(DeviceDetected::detected);
    }

    DeviceDetected device_detected() {
        std::unique_lock<std::mutex> ml(_device_detected_mutex);
        _device_detected_cv.wait(ml, [this] {
            return _device_detected_state != DeviceDetected::uninitialized;
        });

        return _device_detected_state;
    }

    std::string device_name() {
        std::lock_guard<std::mutex> ml(_device_communication_mutex);
        assert(!_device_name.empty());
        return _device_name;
    }

    bool is_connected() {
        std::lock_guard<std::mutex> ml(_device_communication_mutex);
        return _is_connected;
    }

    std::string get_port() const {
        return _port;
    }

private:
    std::unique_ptr<serial::Serial> open_port(const std::string &new_port) {
        return std::make_unique<serial::Serial>(new_port);
    }

    bool check_port(serial::Serial *device) {
        return true;
    }

    void set_device_detected(DeviceDetected state) {
        {
            std::lock_guard<std::mutex> ml(_device_detected_mutex);
            _device_detected_state = state;
            _is_connected = true;
        }
        _device_detected_cv.notify_one();
    }

    void set_device_name(const std::string &name)
    {
        std::lock_guard<std::mutex> ml(_device_communication_mutex);
        _device_name = name;
    }

    std::string _port;
    std::string _device_name;
    bool _is_connected = false;
    std::mutex _device_communication_mutex;
    std::mutex _device_detected_mutex;
    std::condition_variable _device_detected_cv;
    DeviceDetected _device_detected_state = DeviceDetected::uninitialized;
    std::unique_ptr<serial::Serial> _device_handle;
};