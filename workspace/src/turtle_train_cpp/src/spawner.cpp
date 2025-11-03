#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class SpawnerNode : public rclcpp::Node
{
public:
    SpawnerNode() : Node("spawner_node"), spawn_complete_(false)
    {
        // Declare parameters
        this->declare_parameter("canvas_width", 11.088);
        this->declare_parameter("canvas_height", 11.088);
        this->declare_parameter("seed", 42);
        this->declare_parameter("turtle_names", std::vector<std::string>{"follower0"});

        // Set random seed
        int seed = this->get_parameter("seed").as_int();
        rng_.seed(seed);

        // Create spawn service client
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        while (!spawn_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for /spawn service...");
        }

        // Create spawn status service
        spawn_status_service_ = this->create_service<std_srvs::srv::Trigger>(
            "spawn_status",
            std::bind(&SpawnerNode::spawn_status_callback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Spawn all turtles
        auto turtle_names = this->get_parameter("turtle_names").as_string_array();
        for (const auto& name : turtle_names) {
            double x = random_double() * this->get_parameter("canvas_width").as_double();
            double y = random_double() * this->get_parameter("canvas_height").as_double();
            double theta = random_double() * 2.0 * M_PI;
            spawn_turtle(name, x, y, theta);
        }

        spawn_complete_ = true;
    }

private:
    void spawn_turtle(const std::string& name, double x, double y, double theta)
    {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = name;
        request->x = x;
        request->y = y;
        request->theta = theta;

        auto future = spawn_client_->async_send_request(request);
        
        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Spawned turtle: %s", future.get()->name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to spawn turtle: %s", name.c_str());
        }
    }

    void spawn_status_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        response->success = spawn_complete_;
        response->message = "";
    }

    double random_double()
    {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        return dist(rng_);
    }

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr spawn_status_service_;
    bool spawn_complete_;
    std::mt19937 rng_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}