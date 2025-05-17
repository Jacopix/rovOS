#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <opencv2/opencv.hpp>

#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace std::chrono_literals;
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using CallbackReturn = LifecycleNodeInterface::CallbackReturn;

class CameraNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit CameraNode() : LifecycleNode("camera_lifecycle_node"), frame_counter_(0), total_frames_(0)
    {
        RCLCPP_INFO(get_logger(), "Acqusition node instance created.");
    }

protected:

    /**
     * CONFIGURE
     * - check input parameters
     * - define QoS
     * - define publisher
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "Configuring node...");
        last_state_ = this->get_current_state().label();

        // Declare all parameters passed throgh command line
        if( !this->has_parameter("URI") )
            uri_ = this->declare_parameter<std::string>("URI", "");
        if( !this->has_parameter("quality") )
            quality_ = this->declare_parameter<int>("quality", 90);
        if( !this->has_parameter("frame_logged") )
            frame_logged_ = this->declare_parameter<int>("frame_logged", 10);
        if( !this->has_parameter("fps") )
            fps_ = this->declare_parameter<int>("fps", 0);
        if( !this->has_parameter("compression_format") )
            compression_format_ = this->declare_parameter<std::string>("compression_format", ".jpg");

        // Check URI
        if (uri_.empty()) {
            RCLCPP_ERROR(get_logger(), "Camera URI must not be empty.");
            // If URI is empty, stay in UNCONFIGURED
            return CallbackReturn::FAILURE;
        }

        // Setup QoS
        //   KeepLast(1) -> keeps in memory only last message
        //   rmw_qos_profile_sensor_data -> standard profile for sensor data
        auto qos_ = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        // Setup publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("ip_stream_image", qos_);
        RCLCPP_INFO(get_logger(), "Node configured");
        return CallbackReturn::SUCCESS;
    }

    /**
     * ACTIVATE
     * - check for camera
     * - define fps
     * - start timer
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "Activating camera...");
        last_state_ = this->get_current_state().label();

        // Activate publisher
        publisher_->on_activate();

        // Camera not working, stay in INACTIVE state
        if (!start_camera()) {
            return CallbackReturn::FAILURE;
        }

        // Custom fps not set. Autodetect fps from camera
        if (fps_ <= 0) {
            fps_ = static_cast<int>(cap_.get(cv::CAP_PROP_FPS));
        }

        // Audodetect failed -> use default fps value
        if (fps_ <= 0) {
            RCLCPP_ERROR(get_logger(), "FPS could not be determined. Setting default value of 15fps");
            fps_ = 15;
            return CallbackReturn::FAILURE;
        }

        // Setup callback and timer
        auto callback_ = std::bind(&CameraNode::timer_callback, this);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / fps_), callback_);

        RCLCPP_INFO(get_logger(), "Node activated");
        return CallbackReturn::SUCCESS;
    }

    /**
     * DEACTIVATE
     * - reset timer
     * - release camera
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "Deactivating node...");
        last_state_ = this->get_current_state().label();
        
        // Stop timer, close camera, stop publisher
        timer_.reset();
        if (cap_.isOpened()) {
            cap_.release();
        }
        publisher_->on_deactivate();

        return CallbackReturn::SUCCESS;
    }

    /**
     * CLEANUP
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "Cleaning up resources...");
        last_state_ = this->get_current_state().label();

        return CallbackReturn::SUCCESS;
    }

    /**
     * SHUTDOWN
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "Shutting down node...");
        last_state_ = this->get_current_state().label();

        return on_cleanup(get_current_state());
    }

    // ERROR PROCESSING
    // CallbackReturn on_error(const rclcpp_lifecycle::State &) override
    // {
    //     RCLCPP_ERROR(get_logger(), "Node entered error state.");
    //     if(last_state_.compare("configuring") == 0)
    //     {
    //         RCLCPP_ERROR(get_logger(), "Failed while configuring => unconfigured");
    //         return CallbackReturn::SUCCESS;
    //     }
    //     else if(last_state_.compare("inactive") == 0)
    //     {
    //         RCLCPP_ERROR(get_logger(), "Failed while inactive: cleaning up");
    //         this->cleanup();
    //     }
    //     else if(last_state_.compare("activating") == 0)
    //     {
    //         RCLCPP_ERROR(get_logger(), "Failed while inactive: cleaning up");
    //         this->cleanup();
    //     }

    //     return CallbackReturn::FAILURE;
    // }

private:
    bool start_camera()
    {
        cap_.open(uri_);
       // auto start_time = std::chrono::steady_clock::now();

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(get_logger(), "Could not open camera stream.");
            return false;
        }

        return true;
    }

    void timer_callback()
    {
        cv::Mat frame;

        // Try to grab a frame. If it fails, deactivate node 
        if (!cap_.read(frame)) {
            RCLCPP_ERROR(get_logger(), "Failed to grab frame.");
            this->deactivate();
            return;
        }

        // Compress frame
        std::vector<uchar> encoded;
        std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, quality_ };
        try {
            if (!cv::imencode(compression_format_, frame, encoded, params)) {
                RCLCPP_ERROR(get_logger(), "Failed to encode frame.");
                return;
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Encoding exception: %s", e.what());
            this->deactivate();
            return;
        }

        auto msg = sensor_msgs::msg::CompressedImage();
        msg.header.stamp = now();
        msg.format = compression_format_;
        msg.data = encoded;

        publisher_->publish(msg);

        frame_counter_++;
        total_frames_++;

        if (frame_counter_ >= frame_logged_) {
            RCLCPP_INFO(get_logger(), "Published %d frames (total %d)", frame_counter_, total_frames_);
            frame_counter_ = 0;
        }
    }

    // Parameters and resources
    std::string uri_;
    int quality_;
    int frame_logged_;
    int fps_;
    std::string compression_format_;
    std::string last_state_;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;

    int frame_counter_;
    int total_frames_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
