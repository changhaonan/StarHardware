#pragma once
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

namespace star { namespace star_ros {
    using ImageMsg = sensor_msgs::Image;
    typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg> ApproxSyncPolicy_2;
    typedef message_filters::Synchronizer<ApproxSyncPolicy_2> ApproxTimeSynchronizer_2;

    template <class M>
    class BagSubscriber : public message_filters::SimpleFilter<M> {
    public:
        void newMessage(const boost::shared_ptr<M const>& msg) {
            this->signalMessage(msg);
        }
    };
    
}
}