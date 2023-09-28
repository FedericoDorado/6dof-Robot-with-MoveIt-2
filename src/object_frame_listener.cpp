#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/msg/float64_multi_array.hpp> // Incluir el mensaje de matriz

#include "geometry_msgs/msg/quaternion.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;


// Declarar el publicador global
rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transformed_publisher_;

void transformCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
        // Agregar un mensaje de depuración para verificar si la función se llama
    RCLCPP_INFO(rclcpp::get_logger("transform_listener_node"), "Received a transform message");


    // Extraer las coordenadas x, y, y z del mensaje
    double x = msg->transform.translation.x;
    double y = msg->transform.translation.y;
    double z = msg->transform.translation.z;

    double x_roll = msg->transform.rotation.x;
    double y_pitch = msg->transform.rotation.y; 
    double z_yaw = msg->transform.rotation.z;
    double w = msg->transform.rotation.w;

    // Imprimir los valores del cuaternión
    RCLCPP_INFO(rclcpp::get_logger("transform_listener_node"), "x_roll: %.2f", x_roll);
    RCLCPP_INFO(rclcpp::get_logger("transform_listener_node"), "y_pitch: %.2f", y_pitch);
    RCLCPP_INFO(rclcpp::get_logger("transform_listener_node"), "z_yaw: %.2f", z_yaw);
    RCLCPP_INFO(rclcpp::get_logger("transform_listener_node"), "w: %.2f", w);

    Eigen::Quaterniond q;
    q.x() = x_roll;
    q.y() = y_pitch;
    q.z() = z_yaw;
    q.w() = w;
    Eigen::Matrix3d rotation_matrix = q.normalized().toRotationMatrix();

    // La matriz de transformación homogenea se realizao con una rotación respecto al eje 

      // Imprimir la matriz de transformación
// Imprime los elementos de la matriz de transformación
    RCLCPP_INFO(rclcpp::get_logger("transform_listener_node"), "Transform Matrix:");
    for (int i = 0; i < 3; i++) {
        RCLCPP_INFO(rclcpp::get_logger("transform_listener_node"), "%.2f %.2f %.2f",
                    rotation_matrix(i, 0), rotation_matrix(i, 1), rotation_matrix(i, 2));
    }   

    Eigen::Matrix<double, 4, 4> matriz_transformacion_homogenea;
    matriz_transformacion_homogenea << 0, 0, 1, -0.88,
                                    -1, 0, 0, -0.49,
                                    0, -1, 0, 0.09,
                                    0, 0, 0, 1;


    Eigen::Matrix<double, 4, 1> objeto_detectado_respecto_a_la_camara;
    objeto_detectado_respecto_a_la_camara << x,
                                            y,
                                            z,
                                            1;


    // Realizar la multiplicación de matrices
    Eigen::Matrix<double, 4, 1> result_matrix;
    result_matrix = matriz_transformacion_homogenea * objeto_detectado_respecto_a_la_camara;

    double x_transformed = result_matrix(0, 0); // Valor en la primera fila (0) y primera columna (0)
    double y_transformed = result_matrix(1, 0); // Valor en la segunda fila (1) y primera columna (0)
    double z_transformed = result_matrix(2, 0); // Valor en la tercera fila (2) y primera columna (0)

    // if (z_transformed < 0){
    //     z_transformed = 0;
    // }
    // else z_transformed = z_transformed;


    // // Imprimir las coordenadas transformadas utilizando rclcpp::info
    RCLCPP_INFO(rclcpp::get_logger("transform_listener_node"), "X Resultante: %.2f", x_transformed);
    RCLCPP_INFO(rclcpp::get_logger("transform_listener_node"), "Y Resultante: %.2f", y_transformed);
    RCLCPP_INFO(rclcpp::get_logger("transform_listener_node"), "Z Resultante: %.2f", z_transformed);

    // Crear un mensaje TransformStamped
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.frame_id = "Object_frame"; // El frame de referencia original
    rclcpp::Clock clock;
    transform_msg.header.stamp = clock.now();

    // Configurar la traslación
    transform_msg.transform.translation.x = x_transformed;
    transform_msg.transform.translation.y = y_transformed;
    transform_msg.transform.translation.z = z_transformed;

    // Configurar la rotación (puedes usar el cuaternión que ya tienes)
    tf2::Quaternion q1;
    q1.setRPY(x_roll, y_pitch, z_yaw); // Configura la rotación utilizando ángulos de Euler
    tf2::convert(q1, transform_msg.transform.rotation);

    // Publicar el mensaje TransformStamped
    transformed_publisher_->publish(transform_msg);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto transform_listener_node = rclcpp::Node::make_shared("transform_listener_node", node_options);
    auto transform_publisher_node = rclcpp::Node::make_shared("transform_publisher_node", node_options);

    // Crear un suscriptor para el mensaje de transformación en el topic /darknet_ros_3d/object_frame
    auto transform_subscriber = transform_listener_node->create_subscription<geometry_msgs::msg::TransformStamped>(
        "/darknet_ros_3d/object_frame", 10, transformCallback);

    // // Crear un publicador para los valores transformados (usando la variable global)
    // transformed_values_publisher = transform_publisher_node->create_publisher<std_msgs::msg::Float64MultiArray>(
    //     "/object_coordinates_respect_the_robot", 10);

    transformed_publisher_ = transform_publisher_node->create_publisher<geometry_msgs::msg::TransformStamped>(
        "/object_pose_regard_the_robot", 10);

    rclcpp::spin(transform_listener_node); // Mantén el nodo en ejecución
    rclcpp::shutdown();
    return 0;
}
