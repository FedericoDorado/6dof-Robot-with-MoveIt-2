### 6DOF Robot using MoveIt 2
Planning and executing trayectories on UAO'S Centauri 6DOF using ROS 2 and Moveit 2.

![](https://github.com/FedericoDorado/6dof-Robot-with-MoveIt-2/blob/main/Captura%20desde%202024-01-16%2011-09-36.png?raw=true)

[Demo video](https://www.youtube.com/watch?v=_tcTTSba5mw "Demo video")

## Prerequisites
Have previously installed Docker

# Getting Started

To test this package, its installation will be facilitated using Docker, where a container has all the dependencies necessary for its execution. You will need to follow the next steps:
- Do: `docker pull federico99/centauri_ros2_foxy:moveit2`

- Once it is done, in a new terminal write first: `xhost +`

- Then:  `docker run --name centauri_moveit2 -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev --network=host --env=DISPLAY federico99/centauri_ros2_foxy:moveit2`
- When the containter is ready, write: `docker start centauri_moveit2` and `docker exec -it centauri_moveit2 bash`.
- Here the container is ready, source it writting: `sourceros2`
- FInally execute the launch file to play de demo: `ros2 launch centauri_6dof moveit_demo.launch.py`

### How to Use

MoveIt 2 is running on Rviz 2, In the lower left corner you can  change the values. Choose the planning group for the arm or hand, the predefined goals and click on the Pan and Execute button.

Also you can move the robot manually to a new pose using the mause clicking and mantaining on Centauri's hand.
