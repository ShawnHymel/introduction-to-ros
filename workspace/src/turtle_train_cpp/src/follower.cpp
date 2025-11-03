#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class FollowerNode : public rclcpp::Node
{
public:
    FollowerNode() : Node("follower_node")
    {
        // Declare parameters
        this->declare_parameter("debug", false);
        this->declare_parameter("turtle_name", "");
        this->declare_parameter("leader_name", "");
        this->declare_parameter("follow_distance", 1.0);
        this->declare_parameter("follow_period", 0.2);

        // Get parameters
        debug_ = this->get_parameter("debug").as_bool();
        turtle_name_ = this->get_parameter("turtle_name").as_string();
        leader_name_ = this->get_parameter("leader_name").as_string();
        follow_distance_ = this->get_parameter("follow_distance").as_double();

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

        // Delay briefly to prevent missed transform broadcast messages
        rclcpp::sleep_for(2s);

        // Set up TF2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create publisher
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/" + turtle_name_ + "/cmd_vel", 10);

        // Create timer for following
        double period = this->get_parameter("follow_period").as_double();
        auto period_ms = std::chrono::milliseconds(static_cast<int>(period * 1000));
        timer_ = this->create_wall_timer(
            period_ms, std::bind(&FollowerNode::follow, this));

        RCLCPP_INFO(this->get_logger(), "Follower node started");
    }

private:
    void follow()
    {
        // Create a target point 1 unit behind the leader in the leader's frame
        geometry_msgs::msg::PointStamped leader_target;
        leader_target.header.frame_id = leader_name_;
        leader_target.header.stamp = rclcpp::Time(0);  // Use latest available transform
        leader_target.point.x = -1.0 * follow_distance_;
        leader_target.point.y = 0.0;
        leader_target.point.z = 0.0;

        // Look up the transform and transform the point
        geometry_msgs::msg::PointStamped follower_target;
        try {
            follower_target = tf_buffer_->transform(
                leader_target, 
                turtle_name_,
                tf2::durationFromSec(1.0));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            return;
        }

        // Calculate distance and angle to target
        double dx = follower_target.point.x;
        double dy = follower_target.point.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        double angle = std::atan2(dy, dx);

        // Create and publish command
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 1.0 * dist;
        msg.angular.z = 2.0 * angle;
        cmd_pub_->publish(msg);

        // Debug output
        if (debug_) {
            RCLCPP_INFO(this->get_logger(), 
                       "dx=%.2f, dy=%.2f, dist=%.2f, angle=%.2f",
                       dx, dy, dist, angle);
        }
    }

    bool debug_;
    std::string turtle_name_;
    std::string leader_name_;
    double follow_distance_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}