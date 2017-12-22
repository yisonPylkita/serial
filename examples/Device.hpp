#pragma once
#include <string>
#include <memory>
#include <serial/serial.h>


namespace nlohmann {
typedef std::map<std::string, std::string> json;
}

struct Device
{


    private:
//        std::deque<std::vector<byte_t>> _device_in;
//        std::deque<byte_t> device_out_stream;
    };

    typedef Device * Pointer;
//    using Instance = std::unique_ptr<Device>;
//    using Handle = std::unique_ptr<serial::Serial>;
    typedef std::function<Pointer(serial::Serial *)> create_instance_t;

    Device(const std::string &) {}

    virtual ~Device() = default;

    virtual std::vector<std::string> subsystems() = 0;

    void process_api_request(const std::string &action, const nlohmann::json &details)
    {

    }

    std::vector<byte_t> get_data_from_device()
    {
//        // TODO: check what happens when you want to read a lot of data (above 64 kB)
//        std::vector<byte_t> dev_res;
//        _device_handle->read(dev_res);
//        return dev_res;

        if (!_device_handle->isOpen())
            throw std::runtime_error("Device is no longer available");
        // TODO: probable memory fragmentation
        std::vector<byte_t> incoming;
        while (true) {
            size_t read_bytes = _device_handle->read(incoming, 64);
            if (!read_bytes)
                break;
            device_out_stream += incoming;
        }
    }

    void send_data_to_device(const std::vector<byte_t> &data_to_send)
    {
        // TODO: check what happens when you want to send a lot of data (above 64 kB)
        assert(_device_handle->write(data_to_send) == data_to_send.size());
    }

    virtual std::optional<nlohmann::json> next_response() = 0;

    virtual void device_loop() = 0;

protected:
    void set_device_handle(Handle device_handle)
    {
        _device_handle = std::move(device_handle);
    }

    serial::Serial * get_device_handle()
    {
        return _device_handle.get();
    }

private:
    Handle _device_handle;

};