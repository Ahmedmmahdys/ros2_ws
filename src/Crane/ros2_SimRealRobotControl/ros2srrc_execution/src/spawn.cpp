/*#include <argparse/argparse.hpp>
#include <filesystem>
#include <xacro/xacro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>



int main(int argc, char** argv) {
    // Initialize ROS
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("entity_spawner_rviz_gazebo");

    // Parse command line arguments
    argparse::ArgumentParser parser("Spawn object into our Gazebo world.");
    parser.add_argument("--package").help("Package where URDF/XACRO file is located.").default_value("");
    parser.add_argument("--urdf").help("URDF of the object to spawn.").default_value("");
    parser.add_argument("--name").help("Name of the object to spawn.").default_value("OBJECT");
    parser.add_argument("--namespace").help("ROS namespace to apply to the tf and plugins.").default_value("ros2Grasp");
    parser.add_argument("--ns").help("Whether to enable namespacing").default_value(true);
    parser.add_argument("--x").help("The x component of the initial position [meters].").default_value(0.0f);
    parser.add_argument("--y").help("The y component of the initial position [meters].").default_value(0.0f);
    parser.add_argument("--z").help("The z component of the initial position [meters].").default_value(0.0f);

    try {
        parser.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << parser;
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Creating Service client to connect to `/spawn_entity`");
    auto client = node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    RCLCPP_INFO(node->get_logger(), "Connecting to `/spawn_entity` service...");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Service not available after waiting");
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "...connected!");

    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = parser.get<std::string>("--name");

    std::filesystem::path urdf_file_path = ament_index_cpp::get_package_share_directory(parser.get<std::string>("--package")) / "urdf" / parser.get<std::string>("--urdf");
    xacro::XacroDocument xacro_doc;
    xacro_doc.loadFile(urdf_file_path.string());
    request->xml = xacro_doc.toXML();

    request->initial_pose.position.x = parser.get<float>("--x");
    request->initial_pose.position.y = parser.get<float>("--y");
    request->initial_pose.position.z = parser.get<float>("--z");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    if (parser.get<std::string>("--package") == "ros2srrc_ur10e_gazebo"){
        moveit::planning_interface::MoveGroupInterface group("ur10e_arm");
    }

    if (parser.get<bool>("--ns")) {
        RCLCPP_INFO(node->get_logger(), "spawning `%s` on namespace `%s` at %f, %f, %f",
            request->name.c_str(), parser.get<std::string>("--namespace").c_str(),
            request->initial_pose.position.x, request->initial_pose.position.y, request->initial_pose.position.z);
        request->namespace_ = parser.get<std::string>("--namespace");
    } else {
        RCLCPP_INFO(node->get_logger(), "spawning `%s` at %f, %f, %f",
            request->name.c_str(), request->initial_pose.position.x,
            request->initial_pose.position.y, request->initial_pose.position.z);
    }

    RCLCPP_INFO(node->get_logger(), "Spawning OBJECT using service: `/spawn_entity`");
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Response: %s", result.get()->status_message.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service spawn_entity");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Done! Shutting down node.");
    rclcpp::shutdown();
    return 0;
}*/

