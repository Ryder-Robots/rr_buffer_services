#include "rr_buffer_services/rr_controller.hpp"

using namespace rrobot;

void RrController::init(rr_interfaces::msg::BufferResponse::SharedPtr buffer_response, std::shared_ptr<std::shared_mutex> mutex) {
    buffer_response_ = buffer_response;
    mutex = mutex_;
}

void RrController::reset_response() {
}

void RrController::callback() {

}

void RrController::publish() {

}

