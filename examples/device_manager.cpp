#include <serial/serial.h>
#include <vector>
#include <string>
#include <deque>
#include <memory>
#include <map>
#include <deque>
#include <cassert>
#include <optional>
#include <thread>
#include <mutex>
#include <functional>
#include <atomic>
#include <iostream>
#include "stl_utils.hpp"
#include "Device.hpp"
#include "DeviceManager.hpp"


typedef unsigned char byte_t;
using namespace vec::operators;



namespace devices {
namespace fm30 {

struct Fm30 : public Device::DeviceImpl_CRTP<Fm30>
{
    typedef Device::DeviceImpl_CRTP<Fm30> BaseClass;

//    explicit Fm30(std::unique_ptr<serial::Serial> s) {
//        _device_comunication_thread = std::thread([this] (std::unique_ptr<serial::Serial> device) {
//            while (!_exit_thread) {
//                auto impl = reinterpret_cast<DevImpl *>(this);
//
//                // check if there is a response from device
//
//                const auto res = impl->process_dev_res(device);
//                if (!res.empty())
//                    _responses.push_back(res);
//
//                // check if there is a request from api
//                if (_requests.size() > 0) {
//                    const auto req = _requests.front();
//                    _requests.pop_front();
//                    impl->process_api_req(device, );
//                }
//
//                std::vector<byte_t> dev_res;
//                 device->read(dev_res);
//                if (!dev_res.empty()) {
////                    logger->info("Request \t [{}]", vec::to_string(cf_req, "hex", ""));
////                        Request req = Request::deserialize(req_raw);
//                    // TODO: change this
//                    try {
//                        ->process_res(dev_res);
//                    } catch (const std::exception &ex) {
////                        logger->critical("This exception should never be passed here - {}", ex.what());
////                        assert(false);
//                    }
//                }
//
//                // check if there is a command from carflow-device
//                nlohmann::json
//                if (!_req.try_dequeue(ds_req)) {
//                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
//                } else {
//                    std::vector<byte_t> res;
//                    try {
//                        res = reinterpret_cast<DeviceImpl *>(this)->process_simulator_command_impl(ds_req.action, ds_req.options);
//                    } catch (const std::exception &ex) {
//                        logger->critical("This exception should never be passed here - {}", ex.what());
//                        assert(false);
//                    }
//                    this->_pty_master->write_bytes(res);
//                }
//            }
//
//        });
//    }

    explicit Fm30(const std::string &port_path) : BaseClass(port_path)
    {
        Device::Handle device_handle(new serial::Serial(port_path, 115200));
        device_handle->setTimeout(serial::Timeout::simpleTimeout(0)); // Async mode
        set_device_handle(std::move(device_handle));
    }

    virtual ~Fm30() = default;

    std::vector<std::string> subsystems() override
    {
        return {"code_scanner"};
    }

    static Device::Pointer try_create_instance(const std::string &device_path)
    {
        Device::Pointer instance(new Fm30(device_path));
        instance->send_data_to_device({0x7e, 0x00, 0x07, 0x01, 0x00, 0x2a, 0x02, 0xd8, 0x0f});
        if (!vec::begins_with(instance->get_data_from_device(), {0x02, 0x00, 0x00}))
            return nullptr;

        return instance;
    }

    void device_loop() override
    {
        // 1) device read (and send ACK when needed)
        // 2) api read (and device write when needed)

        while (!exit_device_thread) {
            if (!device_in.empty()) {
                auto cmd = device_in.front();
                // TODO: device may start sending data right now. Handle it
                device->write(cmd);
                device_in.pop_front();
            }
        }
    }

    void process_api_request(const std::string &action, const nlohmann::json &details) override
    {

    }

    std::optional<nlohmann::json> response() override
    {
        return std::nullopt;
    }

//    void send_command(const std::string &action, const nlohmann::json &details)
//    {
//        _requests.push_back()
//    }
//
//    std::optional<nlohmann::json> response() override
//    {
//        if (_requests.empty())
//            return {};
//
//        nlohmann::json last_response;
//        {
//            // TODO: mutex here or lockfree queue
//            last_response = _responses.front();
//            _responses.pop_front();
//        }
//        return last_response;
//    }

private:
    std::atomic_bool _exit_thread;
    std::thread _device_comunication_thread;
    std::deque<nlohmann::json> _requests;
    std::deque<nlohmann::json> _responses;
};
}
}

//using is_valid_port_t = std::function<bool(Device::Pointer)>;
using create_instance_t = std::function<Device::Pointer(const std::string &)>;

using watch_devices_dir_callback_t = std::function<bool(const std::string &device_path)>;
void watch_devices_dir(watch_devices_dir_callback_t on_new_device_detected)
{
    bool exit_y = true;
    std::vector<std::string> connected_ports;
    std::vector<std::string> ignored_ports;

    while (exit_y) {
        // TODO: for now only serial devices are detected
        // TODO: case when port disappear and after that is reconnected should be covered
        const auto available_ports = serial::list_ports();
        for (const auto &port : available_ports) {
            if (vec::contains(connected_ports, port.port) || vec::contains(ignored_ports, port.port))
                continue;
            std::cout << "Detected new port " << port.port << "\n";
            bool valid_device = on_new_device_detected(port.port);
            if (valid_device)
                connected_ports += port.port;
            else
                ignored_ports += port.port;

        }
    }
}

//typedef std::function<bool(serial::Serial *)> protocol_t;
//std::vector<protocol_t> supported_devices_protocols;

int main_impl(const std::vector<std::string> &args)
{
    DeviceManager device_manager;
    device_manager.add_supported_device(devices::fm30::Fm30::try_create_instance);

    std::thread connection_watch(watch_devices_dir, [&device_manager] (const std::string &device_path) -> bool {
        try {
            return device_manager.detect_device_and_connect(device_path);
        } catch (...) {
            std::cout << "No supported device detected at " << device_path << " port\n";
        }

        return false;
    });

    // TODO: watch for api requests
    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    int exit_code = EXIT_FAILURE;
    try {
        exit_code = main_impl({argv, argv + argc});
    } catch (...) {
        std::cout << "Unhandled unrecognized exception" << std::endl;
    }

    return exit_code;
}