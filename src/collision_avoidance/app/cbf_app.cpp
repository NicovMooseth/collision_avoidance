// Entry point of cbf_node (convention rule 2): only creates the node and spins it.
// No logic belongs here.
#include <rclcpp/rclcpp.hpp>

#include "nodes/cbf_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CbfNode>());
    rclcpp::shutdown();
    return 0;
}
