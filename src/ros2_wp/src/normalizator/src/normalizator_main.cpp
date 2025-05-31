#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "normalizator/normalizator_node.hpp"

/**
 * Node that subscribes to all C.V. nodes outputs and
 * normalize their messages in a .json format for the
 * modem
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto norm_node = std::make_shared<NormalizatorNode>("lc_norm_node");

    norm_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    norm_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::executors::SingleThreadedExecutor exectr;
    exectr.add_node(norm_node->get_node_base_interface());
    exectr.spin();

    norm_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    norm_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    norm_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);

    rclcpp::shutdown();
    return 0;
}
