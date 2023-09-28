from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtener la ubicación del paquete de descripción del robot
    robot_description_package = "nombre_del_paquete_de_descripcion_del_robot"
    robot_description_share_path = get_package_share_directory(robot_description_package)

    # Lanzar el controlador del robot
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_package}],
    )

    # Lanzar el planificador MoveIt 2
    moveit_planner_node = Node(
        package="nombre_del_paquete_de_moveit",
        executable="moveit_planner",
        name="moveit_planner",
        output="screen",
        parameters=[
            {"robot_description": robot_description_package},
            {"use_sim_time": False}  # Cambiar a True si se está utilizando simulación
        ],
    )

    # Lanzar el nodo para el sistema de percepción
    perception_node = Node(
        package="nombre_del_paquete_de_percepcion",
        executable="sistema_de_percepcion",
        name="sistema_de_percepcion",
        output="screen",
    )

    # Lanzar el nodo que conecta el sistema de percepción con MoveIt
    perception_to_moveit_node = Node(
        package="nombre_del_paquete_de_conexion_percepcion_moveit",
        executable="perception_to_moveit_node",
        name="perception_to_moveit_node",
        output="screen",
    )

    # Aquí puedes agregar cualquier otro nodo necesario para tu aplicación

    return LaunchDescription([
        robot_state_publisher_node,
        moveit_planner_node,
        perception_node,
        perception_to_moveit_node,
        # Otros nodos si los hay
    ])

