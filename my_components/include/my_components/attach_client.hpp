#ifndef COMPOSITION_ATTACH_CLIENT_HPP_
#define COMPOSITION_ATTACH_CLIENT_HPP_

#include "rclcpp/client.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include "my_components/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"

using GoToLoading = my_components::srv::GoToLoading;
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
    COMPOSITION_PUBLIC
    // Constructor
    explicit AttachClient(const rclcpp::NodeOptions &options);
private:
    // Attributes
    rclcpp::Client<GoToLoading>::SharedPtr _client;
    bool _final_approach;
    // Methods
    void get_params();
    void handle_response(const rclcpp::Client<GoToLoading>::SharedFuture future);
};

}
#endif // COMPOSITION_ATTACH_CLIENT_HPP_