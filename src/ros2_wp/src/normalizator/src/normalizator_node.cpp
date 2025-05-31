/*
 * TODO: remove '\t', '\n' and spaces from results, not needed by the modem
 */

#include "normalizator/normalizator_node.hpp"

NormalizatorNode::NormalizatorNode(std::string name_node) : rclcpp_lifecycle::LifecycleNode(name_node)
{
    this->init_param();
    RCLCPP_INFO(this->get_logger(), "NormalizatorNode created!");
}

void NormalizatorNode::init_param(void)
{
    // set the parameter for this node
    // by default it does not print additional
    // debugging informations
    rcl_interfaces::msg::ParameterDescriptor param_descr_debug = rcl_interfaces::msg::ParameterDescriptor {};
    param_descr_debug.description = this->PARAM_DESCR_DEBUG;
    this->declare_parameter<bool>(this->PARAM_NAME_DEBUG, param_descr_debug);
}

void NormalizatorNode::init_var(void)
{
    // get parameters from config files
    this->debug = this->get_parameter(this->PARAM_NAME_DEBUG).as_bool();
    if(this->debug)
        RCLCPP_INFO(this->get_logger(), "TRUE!");
    else
        RCLCPP_INFO(this->get_logger(), "FALSE!");
        

    // initialize lambda functions
    auto callback_mono_ir = [this](dual_camera_msgs::msg::MonoIR::UniquePtr msg) -> void
    {
        dual_camera_msgs::msg::JsonString output;
        output.data = "\n{\n\t\"mono_ir\": {\n\t\t\"timestamp\": \"" + msg->timestamp + "\",\n\t\t\"label\": \"" + msg->label + "\",\n\t\t\"confidence\": \"" + std::to_string(msg->confidence) + "\",\n\t\t\"box\": {\"" + std::to_string(msg->box[0]) + "\", \"" + std::to_string(msg->box[1]) + "\", \"" + std::to_string(msg->box[2]) + "\", \"" + std::to_string(msg->box[3]) + "\"}\n\t}\n}\n";
        if(this->debug)
        {
            RCLCPP_INFO(this->get_logger(), output.data.c_str());
        }
    };
    auto callback_stereo_vo = [this](dual_camera_msgs::msg::StereoVO::UniquePtr msg) -> void
    {
        dual_camera_msgs::msg::JsonString output;
        output.data = "\n{\n\t\"odometry\": {\n\t\t\"timestamp\": \"" + msg->timestamp + "\",\n\t\t\"translation\": {\"" + std::to_string(msg->translation[0]) + "\", \"" + std::to_string(msg->translation[1]) + "\", \"" + std::to_string(msg->translation[2]) + "\"},\n\t\t\"rotation\": {\"" + std::to_string(msg->rotation[0]) + "\", \"" + std::to_string(msg->rotation[1]) + "\", \"" + std::to_string(msg->rotation[2]) + "\", \"" + std::to_string(msg->rotation[3]) + "\", \"" + std::to_string(msg->rotation[4]) + "\", \"" + std::to_string(msg->rotation[5]) + "\", \"" + std::to_string(msg->rotation[6]) + "\", \"" + std::to_string(msg->rotation[7]) + "\", \"" + std::to_string(msg->rotation[8]) + "\"}\n\t}\n}\n";
        if(this->debug)
        {
            RCLCPP_INFO(this->get_logger(), output.data.c_str());
        }
    };
    auto callback_stereo_ir = [this](dual_camera_msgs::msg::StereoIR::UniquePtr msg) -> void
    {
        dual_camera_msgs::msg::JsonString output;
        output.data = "\n{\n\t\"stereo_ir\": {\n\t\t\"timestamp\": \"" + msg->timestamp + "\",\n\t\t\"label\": \"" + msg->label + "\"\n\t}\n}\n";
        if(this->debug)
        {
            RCLCPP_INFO(this->get_logger(), output.data.c_str());
        }
    };

    // create QoS profiles
    // keep in memory last 5 messages (they small)
    // use default profile
    rclcpp::QoS qos_profile_sub = rclcpp::QoS(rclcpp::KeepLast(this->QOS_DEPTH_SUB), rmw_qos_profile_default);
    rclcpp::QoS qos_profile_pub = rclcpp::QoS(rclcpp::KeepLast(this->QOS_DEPTH_PUB), rmw_qos_profile_default);

    // create subscription for every topic
    this->sub_mono_ir_ = this->create_subscription<dual_camera_msgs::msg::MonoIR>(this->TN_MONO_IR, qos_profile_sub, callback_mono_ir);
    this->sub_stereo_vo_ = this->create_subscription<dual_camera_msgs::msg::StereoVO>(this->TN_STEREO_VO, qos_profile_sub, callback_stereo_vo);
    this->sub_stereo_ir_ = this->create_subscription<dual_camera_msgs::msg::StereoIR>(this->TN_STEREO_IR, qos_profile_sub, callback_stereo_ir);

    // create publisher
    this->pub_json_ = this->create_publisher<dual_camera_msgs::msg::JsonString>(this->TN_MODEM, qos_profile_pub);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NormalizatorNode::on_configure(const rclcpp_lifecycle::State&)
{
    this->init_var();
    RCLCPP_INFO(this->get_logger(), "NormalizatorNode configured!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NormalizatorNode::on_activate(const rclcpp_lifecycle::State&)
{
    this->pub_json_->on_activate();
    RCLCPP_INFO(this->get_logger(), "NormalizatorNode activated!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NormalizatorNode::on_deactivate(const rclcpp_lifecycle::State&)
{
    this->pub_json_->on_deactivate();
    RCLCPP_INFO(get_logger(), "NormalizatorNode deactivated!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NormalizatorNode::on_cleanup(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "NormalizatorNode cleaned up!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NormalizatorNode::on_shutdown(const rclcpp_lifecycle::State&)
{
    pub_json_.reset();
    sub_mono_ir_.reset();
    sub_stereo_ir_.reset();
    sub_stereo_vo_.reset();
    RCLCPP_INFO(get_logger(), "NormalizatorNode shutting down!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
