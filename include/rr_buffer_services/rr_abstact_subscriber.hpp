#ifndef RR_ABSTRACT_SUBSCRIBER_HPP
#define RR_ABSTRACT_SUBSCRIBER_HPP

#include <string>

namespace rrobot {
    class RrABstractSubscriber {

        virtual std::string getTopicParam() = 0;
        virtual std::string getQueueSzParam() = 0;
        virtual std::string getTopicDefault() = 0;
        virtual int getQueueSzDefault() = 0;

        /**
         * @fn set_ctl_node
         * @brief sets the controller node
         * 
         * Controller coordinates subscribers, and performs publising.
         */
        virtual void set_ctl_node(std::shared_ptr<RrSubscriber>  ctl) = 0;
    };
}

#endif // RR_ABSTRACT_SUBSCRIBER_HPP