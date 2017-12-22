#include <cstddef>
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <atomic>
#include <serial/serial.h>
#include <thread>


namespace nlohmann {
typedef std::map<std::string, std::string> json;
}

template <typename DevImpl>
struct DeviceImpl_CRTP
{
    using Handle = std::unique_ptr<serial::Serial>;
    typedef serial::Serial *WeakHandle;

    explicit DeviceImpl_CRTP(const std::string &port_path) = default;

    static void device_loop(DeviceImpl_CRTP *device)
    {
        while (!device->_exit_thread) {
            auto impl = reinterpret_cast<DevImpl *>(device);

            // check if there is a response from device
            std::vector<std::byte> readed_from_device;
            _device_handle->read(readed_from_device, )

            const auto res = impl->process_dev_res(device);
            if (!res.empty())
                _responses.push_back(res);

            // check if there is a request from api
            if (_requests.size() > 0) {
                const auto req = _requests.front();
                _requests.pop_front();
                impl->process_api_req(device,);
            }

            std::vector<byte_t> dev_res;
            device->read(dev_res);
            if (!dev_res.empty()) {
                //                    logger->info("Request \t [{}]", vec::to_string(cf_req, "hex", ""));
                //                        Request req = Request::deserialize(req_raw);
                // TODO: change this
                try {
                    ->process_res(dev_res);
                } catch (const std::exception &ex) {
                    //                        logger->critical("This exception should never be passed here - {}", ex.what());
                    //                        assert(false);
                }
            }

            // check if there is a command from carflow-device
            nlohmann::json
            if (!_req.try_dequeue(ds_req)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            } else {
                std::vector<byte_t> res;
                try {
                    res = reinterpret_cast<DeviceImpl *>(this)->process_simulator_command_impl(ds_req.action,
                                                                                               ds_req.options);
                } catch (const std::exception &ex) {
                    logger->critical("This exception should never be passed here - {}", ex.what());
                    assert(false);
                }
                this->_pty_master->write_bytes(res);
            }
        }
    }

private:
    std::atomic_bool _exit_thread;
    Handle _device_handle;
};