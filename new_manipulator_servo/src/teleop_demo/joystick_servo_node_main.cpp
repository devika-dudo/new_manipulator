#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Load the component
  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("joy_to_twist_publisher", options);
  
  // We need to manually create the component since we can't link directly
  // The component is already registered, we just need to spin
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  
  // Load the JoyToServoPub component
  auto loader = std::make_shared<class_loader::ClassLoader>("libservo_controller_input.so");
  auto node_factory = loader->createInstance<rclcpp_components::NodeFactory>("rclcpp_components::NodeFactoryTemplate<moveit_servo::JoyToServoPub>");
  auto joy_node = node_factory->create_node_instance(options);
  
  exec->add_node(joy_node.get_node_base_interface());
  exec->spin();
  
  rclcpp::shutdown();
  return 0;
}
