#include "aws_robomaker_gazebo_bridge/AwsPerfMetrics.h"
#include "boost/shared_ptr.hpp"
#include "gazebo/gazebo.hh"
#include "gazebo/gazebo_client.hh"
#include "ros/ros.h"


using PerfMetrics = aws_robomaker_gazebo_bridge::AwsPerfMetrics;

const std::string kDefaultGazeboPerfMetricsTopic = "/gazebo/aws/perf_metrics";
const std::string kDefaultGazeboWorldStatsTopic = "/gazebo/default/world_stats";

namespace {
    // TODO: This calculation is wrong, since it doesn't account for times when
    //  the simulation is paused, or changes in RTF over time. Need to
    //  implement a better filter, but for now
    double real_time_factor(const gazebo::common::Time &simTime, const gazebo::common::Time &realTime) {
        if (realTime == gazebo::common::Time::Zero) {
            return 0.0;
        }
        return (simTime / realTime).Double();
    }
}

namespace robomaker {
    class PerfMetricsBridge {

    public:
        PerfMetricsBridge() : nh_("~") {
            std::string world_stats_topic, perf_metrics_topic;
            nh_.param<std::string>("gazebo_perf_metrics_topic", perf_metrics_topic, kDefaultGazeboPerfMetricsTopic);
            nh_.param<std::string>("gazebo_world_stats_topic", world_stats_topic, kDefaultGazeboWorldStatsTopic);

            perf_metrics_pub_ = nh_.advertise<PerfMetrics>(perf_metrics_topic, 1);

            gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
            gazebo_node_->Init();
            gazebo_world_stats_sub_ = gazebo_node_->Subscribe(world_stats_topic, &PerfMetricsBridge::publish_perf_metrics, this);
        }

        void publish_perf_metrics(ConstWorldStatisticsPtr& msg) {
            PerfMetrics metrics;
            metrics.rtf = real_time_factor(
                gazebo::msgs::Convert(msg->sim_time()),
                gazebo::msgs::Convert(msg->real_time())
            );
            perf_metrics_pub_.publish(metrics);
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher perf_metrics_pub_;
        boost::shared_ptr<gazebo::transport::Node> gazebo_node_;
        gazebo::transport::SubscriberPtr gazebo_world_stats_sub_;
    };
}

int main(int argc, char **argv) {
    gazebo::client::setup(argc, argv);
    ros::init(argc, argv, "aws_perf_metrics_bridge");
    robomaker::PerfMetricsBridge bridge;
    ros::spin();
    return 0;
}
