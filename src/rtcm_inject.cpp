#include <mavros/mavros_plugin.h>
#include <ros/console.h>
#include <boost/array.hpp>
#include <std_msgs/UInt8MultiArray.h>
#include <cmath>

namespace mavros {
namespace extra_plugins {

using mavros::UAS;

class RTCMInjectPlugin : public plugin::PluginBase 
{
public:
    RTCMInjectPlugin() : PluginBase(),
        rtk_nh("~")
    { }

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
        rtcm_sub = rtk_nh.subscribe("/rtk_000001/rtcm_stream", 10, &RTCMInjectPlugin::rtcm_cb, this);
    }

    Subscriptions get_subscriptions()
    {
        return{};
    }

private:
    ros::NodeHandle rtk_nh;
    ros::Subscriber rtcm_sub;

    uint8_t counter = 0;
    const int max_size = 180;
    int rtcm_seq = 0;

    void rtcm_cb(const std_msgs::UInt8MultiArray &rtcm)
    {
        if (!rtcm.data.size()) return;

        int fragments = ceil(rtcm.data.size()/(float)max_size); // 180 - max size of GPS_RTCM_DATA mavlink data            
        
        for (auto frag_no = 0; frag_no < fragments; frag_no++)
        {
            using mavlink::common::msg::GPS_RTCM_DATA;
            mavlink::common::msg::GPS_RTCM_DATA message{};
            message.flags = ((fragments == 1) ? 0 : 1) | (frag_no << 1) | (rtcm_seq << 3);

            message.len = (frag_no == fragments-1) ? (rtcm.data.size() % max_size) : max_size;
            for (auto i = 0; i < message.len; i++)
                message.data[i] = rtcm.data[i + frag_no*max_size];
            UAS_FCU(m_uas)->send_message_ignore_drop(message);
        }
        rtcm_seq++;
        rtcm_seq %= 32;
        if (counter > 20)   // 20 messages == 5 seconds
        {
            ROS_INFO("RTK online");
            counter = 0;
        }
        counter++;
    }
};
}   // namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::RTCMInjectPlugin, mavros::plugin::PluginBase)
