#pragma once
#include <cassert>
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <serial/serial.h>


struct DeviceThread
{
    enum class DeviceDetected {
        uninitialized,
        not_detected,
        detected
    };

    using Instance = std::unique_ptr<DeviceThread>;

    //    DeviceThread(const DeviceThread &) = delete;
    //    void operator=(const DeviceThread &) = delete;

    virtual void device_loop() = 0;
    virtual DeviceDetected device_detected() = 0;
    virtual std::string device_name() = 0;
    virtual bool is_connected() = 0;
    virtual std::string get_port() const = 0;
};

template <typename DevImpl>
struct DeviceThread_CRTP : public DeviceThread
{
    explicit DeviceThread_CRTP(const std::string &port) {
        _port = port;
    }

    void device_loop() override {
        auto dev_impl = static_cast<DevImpl *>(this);
        if (!dev_impl->initialize_device())
            set_device_detected(DeviceDetected::not_detected);
        set_device_name(dev_impl->device_name_s());
        set_device_detected(DeviceDetected::detected);
    }

    DeviceDetected device_detected() override {
        std::unique_lock<std::mutex> ml(_device_detected_mutex);
        _device_detected_cv.wait(ml, [this] {
            return _device_detected_state != DeviceDetected::uninitialized;
        });

        return _device_detected_state;
    }

    std::string device_name() override {
        std::lock_guard<std::mutex> ml(_device_communication_mutex);
        assert(!_device_name.empty());
        return _device_name;
    }

    bool is_connected() override {
        std::lock_guard<std::mutex> ml(_device_communication_mutex);
        return _is_connected;
    }

    std::string get_port() const override {
        return _port;
    }

private:

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
};

namespace devices {
struct Fm30 : public DeviceThread_CRTP<Fm30>
{
    static std::string device_name_s()
    {
        return "fm30";
    }

    explicit Fm30(const std::string &port) : DeviceThread_CRTP<Fm30>(port)
    {
        _port = port;
    }


    bool initialize_device()
    {
        // open port
        _device_handle = std::make_unique<serial::Serial>(_port);
        if (!_device_handle)
            return false;

        return check_port();
    }

private:
    bool check_port()
    {
        // TODO: implement this
        return true;
    }

    std::unique_ptr<serial::Serial> _device_handle;
    std::string _port;
};
}