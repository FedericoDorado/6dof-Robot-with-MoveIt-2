#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/msg/float64_multi_array.hpp> 

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <cmath>
#include <iostream>
#include <thread> // Incluir la biblioteca para trabajar con hilos
#include <chrono> // Incluir la biblioteca para manejar el tiempo

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");

double to_radians(const double deg_angle)
{
  return deg_angle * M_PI / 180.0;
}

// Declarar las variables globales para X, Y y Z del objeto detectado
double x_object = 0.0;
double y_object = 0.0;
double z_object = 0.0;

double x_roll = 0.0;
double y_pitch = 0.0;
double z_yaw = 0.0;
double w = 0.0; 

bool object_pose_received = false;

void get_object_pose(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("object_pose_subscriber_node"), "Received object pose");

  // Extraer las coordenadas x, y, y z del mensaje
  x_object = msg->transform.translation.x;
  y_object = msg->transform.translation.y;
  z_object = msg->transform.translation.z;

  // Extraer la orientación del ojeto
  x_roll = msg->transform.rotation.x;
  y_pitch = msg->transform.rotation.y;
  z_yaw = msg->transform.rotation.z;
  w = msg->transform.rotation.w;

    RCLCPP_INFO(rclcpp::get_logger("object_pose_subscriber_node"), "X Resultante: %.2f", x_object);
    RCLCPP_INFO(rclcpp::get_logger("object_pose_subscriber_node"), "Y Resultante: %.2f", y_object);
    RCLCPP_INFO(rclcpp::get_logger("object_pose_subscriber_node"), "Z Resultante: %.2f", z_object);

  object_pose_received = true;

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);
  
    // Creación de un suscriptor para el mensaje de tipo Float64MultiArray en el topic "/darknet_ros_3d/object_frame"
  auto object_pose_subscriber_node = rclcpp::Node::make_shared("object_pose_subscriber_node", node_options);
  auto object_pose_subscriber = object_pose_subscriber_node->create_subscription<geometry_msgs::msg::TransformStamped>(
      "/object_pose_regard_the_robot", 10, get_object_pose);
  
    // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

    // Esperar a que se reciba al menos un mensaje antes de continuar
    while (!object_pose_received && rclcpp::ok()) {
        rclcpp::spin_some(object_pose_subscriber_node);
    }

  moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_arm_node, "centauri_arm");
  move_group_arm.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_gripper_node, "centauri_hand");
  move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group_arm =
  move_group_arm.getCurrentState()->getJointModelGroup("centauri_arm");

  const moveit::core::JointModelGroup* joint_model_group_hand =
  move_group_gripper.getCurrentState()->getJointModelGroup("centauri_hand");

  // Visualization
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_arm_node, "link0", "move_group_arm_node",
                                                      move_group_arm.getRobotModel());

  moveit_visual_tools::MoveItVisualTools visual_tools_hand(move_group_gripper_node, "joint_gear1", "move_group_gripper_node",
                                                      move_group_gripper.getRobotModel());

  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script */
  /* via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group_arm.getJointModelGroupNames().begin(), move_group_arm.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // INICIO DE LA RUTINA DE PICK AND PLACE

  move_group_arm.setNamedTarget("cero");
  move_group_arm.move();

  move_group_gripper.setNamedTarget("open");
  move_group_gripper.move();

  // Pausar durante 9 segundos
  std::this_thread::sleep_for(std::chrono::seconds(3));

            // SIMULACIÓN DE UN OBJETO PARA AGARRAR

  moveit_msgs::msg::CollisionObject object_to_attach;
  object_to_attach.id = "bottle";
  shape_msgs::msg::SolidPrimitive primitive;
  shape_msgs::msg::SolidPrimitive cylinder_primitive;
  cylinder_primitive.type = primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.15;
  cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.01;

 // Obtener la pose actual del objeto en el sistema de coordenadas del robot
  object_to_attach.header.frame_id = "base_link";
  geometry_msgs::msg::Pose object_pose_in_robot_frame;
  object_pose_in_robot_frame.position.x = x_object;
  object_pose_in_robot_frame.position.y = y_object;
  object_pose_in_robot_frame.position.z = z_object;
  object_pose_in_robot_frame.orientation.w = 1.0;
  object_to_attach.primitives.push_back(cylinder_primitive);
  object_to_attach.primitive_poses.push_back(object_pose_in_robot_frame);
  object_to_attach.operation = object_to_attach.ADD;
  planning_scene_interface.applyCollisionObject(object_to_attach);

//     geometry_msgs::msg::Pose pre_target_pose;
//   tf2::Quaternion q;
//   // target_pose.orientation.w = 1.0;
//   // target_pose.position.x = -0.03;
//   // target_pose.position.y = -0.45; // VALORES QUE FUNCIONAN PARA EJECUTAR PICK AND PLACE 
//   // target_pose.position.z = 0.05;  //  q1.setRPY(to_radians(90), to_radians(0), to_radians(0));
//   pre_target_pose.position.x = x_object;
//   pre_target_pose.position.y = y_object +0.05;
//   pre_target_pose.position.z = z_object;
//   q.setRPY(to_radians(90), to_radians(0), to_radians(0));
//   // q1.setX(x_roll);
//   // q1.setY(y_pitch);
//   // q1.setZ(z_yaw);
//   // q1.setW(w);
//   // Configura una orientación neutra (orientación inicial)
//   // target_pose.orientation.w = 1.0;
//   pre_target_pose.orientation = tf2::toMsg(q);
//   move_group_arm.setPoseTarget(pre_target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

// // if (!success)
// // {
// //     RCLCPP_ERROR(LOGGER, "Failed to plan the trajectory. Exiting...");
// //     rclcpp::shutdown(); // Cerrar el nodo ROS
// //     return 0; // Terminar la ejecución del programa con un código de error.
// // }


//   // move_group_arm.setStartStateToCurrentState();
//   success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_arm);
//   visual_tools.trigger();
//   move_group_arm.move();

  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q1;
  target_pose.orientation.w = 1.0;
  // target_pose.position.x = -0.03;
  // target_pose.position.y = -0.45; // VALORES QUE FUNCIONAN PARA EJECUTAR PICK AND PLACE 
  // target_pose.position.z = 0.05;  //  q1.setRPY(to_radians(90), to_radians(0), to_radians(0));
  target_pose.position.x = x_object;
  target_pose.position.y = y_object;
  target_pose.position.z = z_object;
  q1.setRPY(to_radians(90), to_radians(0), to_radians(0));
  // q1.setX(x_roll);
  // q1.setY(y_pitch);
  // q1.setZ(z_yaw);
  // q1.setW(w);
  // Configura una orientación neutra (orientación inicial)
  // target_pose.orientation.w = 1.0;
  target_pose.orientation = tf2::toMsg(q1);
  move_group_arm.setPoseTarget(target_pose);

  success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

// if (!success)
// {
//     RCLCPP_ERROR(LOGGER, "Failed to plan the trajectory. Exiting...");
//     rclcpp::shutdown(); // Cerrar el nodo ROS
//     return 0; // Terminar la ejecución del programa con un código de error.
// }


  // move_group_arm.setStartStateToCurrentState();
  RCLCPP_INFO(LOGGER, "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_arm);
  visual_tools.trigger();
  move_group_arm.move();

  // Pausar durante 9 segundos
  std::this_thread::sleep_for(std::chrono::seconds(9));

  move_group_gripper.setNamedTarget("closed");
  move_group_gripper.move();

  // std::vector<std::string> touch_links;
  // touch_links.push_back("pinza1");
  // // touch_links.push_back("pinza2");
  // move_group_arm.attachObject(object_to_attach.id,"centauri_hand", touch_links);
  visual_tools.trigger();

  std::this_thread::sleep_for(std::chrono::seconds(5));

  move_group_arm.setNamedTarget("safe");
  move_group_arm.move(); 

  std::this_thread::sleep_for(std::chrono::seconds(7));

  move_group_arm.setStartStateToCurrentState();
  move_group_arm.setNamedTarget("goal");
  success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_arm);
  visual_tools.trigger();
  move_group_arm.move();

  // Pausar durante 5 segundos
  std::this_thread::sleep_for(std::chrono::seconds(9));

  move_group_gripper.setNamedTarget("open");
  move_group_gripper.move();
  // move_group_arm.detachObject(object_to_attach.id);

  // Pausar durante 2 segundos
  std::this_thread::sleep_for(std::chrono::seconds(5));

  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  std::vector<std::string> object_ids;
  object_ids.push_back(object_to_attach.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // END_TUTORIAL
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

    // Ejecutar el nodo para procesar los mensajes
  rclcpp::shutdown();
  return 0;
}