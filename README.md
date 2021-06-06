# neuralnode
### Autonomous Drone Operation Platform
- Simulate --> Verify --> Deploy --> Scale Pipeline
- Scale upto 100 drone swarm members
- For Hardware Test use sim_time = real
## nnDrone

<img src=doc/images/nngaz.png>
<img src=doc/images/nn1.png width="165" height="150"><img src=doc/images/nn2.png width="165" height="150"><img src=doc/images/nn3.png width="165" height="150">

## Setup
Install ROS melodic
Install MAVROS 
Clone Ardupilot Repository

export GAZEBO_MODEL_PATH=/home/chirag/ros_ws/src/neuralnode/src/gazebo_apriltag/models:$GAZEBO_MODEL_PATH
## Run
roslaunch nndrone_simulation nndrone_simulation.launch

rosrun nndrone_simulation sitl

