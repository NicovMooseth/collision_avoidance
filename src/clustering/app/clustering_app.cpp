// Entry point of clustering_node (convention rule 2): only creates the node and spins it.
// No logic belongs here.
#include <rclcpp/rclcpp.hpp>

#include "nodes/clustering_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClusteringNode>());
    rclcpp::shutdown();
    return 0;
}
