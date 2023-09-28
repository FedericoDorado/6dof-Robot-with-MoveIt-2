import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['joint1','joint2','joint3','joint4','joint5','joint6', 'joint_gear1', 'joint_pinza1', 'joint_gear2', 'joint_pinza2', 		'joint_pivot1', 'joint_pivot2',]
        self.goal_positions = [0.8,-0.5,-0.0,0.0,0.0,0.0,0.0,-0.0,0.0,0.0,0.0,0.0]


    def timer_callback(self):
        centauri_trajectory_msg = JointTrajectory()
        centauri_trajectory_msg.joint_names = self.joints
        ## creating a point
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        ## adding newly created point into trajectory message
        centauri_trajectory_msg.points.append(point)
        self.trajectory_publihser.publish(centauri_trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()

    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
