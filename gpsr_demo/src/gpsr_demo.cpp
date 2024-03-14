#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("gpsr_demo_node");
    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    factory.registerFromPlugin(loader.getOSName("follow_person_bt_node"));
    factory.registerFromPlugin(loader.getOSName("count_bt_node"));
    factory.registerFromPlugin(loader.getOSName("describe_object_bt_node"));
    factory.registerFromPlugin(loader.getOSName("describe_person_bt_node"));
    factory.registerFromPlugin(loader.getOSName("find_object_bt_node"));
    factory.registerFromPlugin(loader.getOSName("find_person_bt_node"));
    factory.registerFromPlugin(loader.getOSName("listen_bt_node"));
    factory.registerFromPlugin(loader.getOSName("look_around_bt_node"));
    factory.registerFromPlugin(loader.getOSName("look_to_bt_node"));
    factory.registerFromPlugin(loader.getOSName("move_to_bt_node"));
    factory.registerFromPlugin(loader.getOSName("offer_bt_node"));
    factory.registerFromPlugin(loader.getOSName("pick_object_bt_node"));
    factory.registerFromPlugin(loader.getOSName("place_object_bt_node"));
    factory.registerFromPlugin(loader.getOSName("query_bt_node"));
    factory.registerFromPlugin(loader.getOSName("recognize_person_bt_node"));
    factory.registerFromPlugin(loader.getOSName("speak_bt_node"));
    std::string pkgpath = ament_index_cpp::get_package_share_directory("gpsr_demo");
    std::string xml_file = pkgpath + "/bt_xml/gpsr_demo.xml";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);

    

    BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

    auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 2666, 2667);

    rclcpp::Rate rate(10);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    bool finish = false;
    while (!finish && rclcpp::ok()) {
    status = tree.rootNode()->executeTick();
    finish = (status == BT::NodeStatus::SUCCESS) ||
        (status == BT::NodeStatus::FAILURE);

    rclcpp::spin_some(node);
    rate.sleep();
    }
    std::cout << "GPSR demo Finished with status: " << status << std::endl;
    rclcpp::shutdown();
    return 0;
}