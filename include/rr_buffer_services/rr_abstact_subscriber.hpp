#ifndef RR_ABSTRACT_SUBSCRIBER_HPP
#define RR_ABSTRACT_SUBSCRIBER_HPP

#include <string>

namespace rrobot {
    class RrABstractSubscriber {

        virtual std::string getTopicParam() = 0;
        virtual std::string getQueueSzParam() = 0;
        virtual std::string getTopicDefault() = 0;
        virtual int getQueueSzDefault() = 0;
    };
}

#endif // RR_ABSTRACT_SUBSCRIBER_HPP