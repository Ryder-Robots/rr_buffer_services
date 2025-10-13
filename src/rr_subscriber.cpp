#include "rr_buffer_services/rr_subscriber.hpp"

using namespace rrobot;

void RrSubscriber::init(rr_interfaces::msg::BufferResponse::SharedPtr buffer_response, std::shared_ptr<std::shared_mutex> mutex) {
    buffer_response_ = buffer_response;
    mutex = mutex_;
}

void RrSubscriber::reset_response() {

}

void RrSubscriber::callback() {

}

void RrSubscriber::publish() {

}

