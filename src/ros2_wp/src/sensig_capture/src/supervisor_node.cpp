#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;

class LifecycleMonitor : public rclcpp::Node
{
public:
    LifecycleMonitor()
    : Node("lifecycle_monitor")
    {
        get_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/camera1/camera1_node/get_state");
        change_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/camera1/camera1_node/change_state");

        timer_ = this->create_wall_timer(1s, std::bind(&LifecycleMonitor::check_state, this));
    }

private:
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void check_state()
    {
        if (!get_state_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for get_state service...");
            return;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future = get_state_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 2s) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call get_state service.");
            return;
        }

        auto state = future.get()->current_state.id;
        RCLCPP_INFO(this->get_logger(), "Current state: %u", state);

        switch (state)
        {
            case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
                activate_node();
                break;

            case lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING:
                RCLCPP_FATAL(this->get_logger(), "Node is in ERROR state. Exiting.");
                rclcpp::shutdown();
                break;

            default:
                break;
        }
    }

    void activate_node()
    {
        if (!change_state_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for change_state service...");
            return;
        }

        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

        auto future = change_state_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 2s) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call change_state service.");
            return;
        }

        if (future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Node successfully activated.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to activate node.");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LifecycleMonitor>());
    rclcpp::shutdown();
    return 0;
}
