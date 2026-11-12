// Entry point of scan_to_constraint_node (convention rule 2): only creates the node and spins it.
// No logic belongs here.
#include <rclcpp/rclcpp.hpp>

#include "nodes/scan_to_constraint_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToConstraintNode>());
    rclcpp::shutdown();
    return 0;
}
