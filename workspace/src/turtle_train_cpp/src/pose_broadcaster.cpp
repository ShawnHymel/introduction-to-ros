#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/trigger.hpp"
#include "turtle_train_cpp/util.hpp"

using namespace std::chrono_literals;

class PoseBroadcaster : public rclcpp::Node
{
public:
    PoseBroadcaster() : Node("pose_broadcaster")
    {
        // Declare parameters
        this->declare_parameter("turtle_names", std::vector<std::string>{"turtle1"});
        this->declare_parameter("broadcast_period", 0.1);

        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Wait for spawn_status service
        auto spawn_status_client = this->create_client<std_srvs::srv::Trigger>("spawn_status");
        while (!spawn_status_client->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for /spawn_status service...");
        }

        // Query spawn_status until complete
        while (rclcpp::ok()) {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = spawn_status_client->async_send_request(request);
            
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                if (future.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "Spawner says we're ready!");
                    break;
                }
            }
            rclcpp::sleep_for(2s);
        }

        // Subscribe to all turtle poses
        auto turtle_names = this->get_parameter("turtle_names").as_string_array();
        for (const auto& name : turtle_names) {
            auto callback = [this, name](const turtlesim::msg::Pose::SharedPtr msg) {
                this->pose_callback(msg, name);
            };
            
            auto sub = this->create_subscription<turtlesim::msg::Pose>(
                "/" + name + "/pose", 10, callback);
            subscriptions_.push_back(sub);
        }

        // Create timer for broadcasting
        double period = this->get_parameter("broadcast_period").as_double();
        auto period_ms = std::chrono::milliseconds(static_cast<int>(period * 1000));
        timer_ = this->create_wall_timer(
            period_ms, std::bind(&PoseBroadcaster::broadcast, this));

        RCLCPP_INFO(this->get_logger(), "Transform broadcaster node has started");
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg, const std::string& turtle_name)
    {
        poses_[turtle_name] = *msg;
    }

    void broadcast()
    {
        for (const auto& [name, pose] : poses_) {
            geometry_msgs::msg::TransformStamped t;
            
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "world";
            t.child_frame_id = name;
            
            t.transform.translation.x = pose.x;
            t.transform.translation.y = pose.y;
            t.transform.translation.z = 0.0;
            
            auto q = turtle_train_cpp::euler_to_quaternion(0, 0, pose.theta);
            t.transform.rotation.x = q[0];
            t.transform.rotation.y = q[1];
            t.transform.rotation.z = q[2];
            t.transform.rotation.w = q[3];
            
            tf_broadcaster_->sendTransform(t);
        }
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::vector<rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr> subscriptions_;
    std::map<std::string, turtlesim::msg::Pose> poses_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}