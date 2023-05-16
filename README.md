## Team 1 assignment for COM3528 - Cognitive and Biomimetic Robotics 

The code has been optimised to work on the physical Miros and as a result, may not perform as expected in the simulation.

Running the code:

Cloning the repository in the correct directory.

Navigate to the catkin_ws/src directory using:

cd catkin_ws/src

and clone the project using:

git clone https://github.com/charalambosG-9/team-1-robotics.git

Navigate into the cloned directory using:

cd team-1-robotics

and run the following commands:

chmod +x src/team1.py (to make the file executable)

catkin build

source ~/catkin_ws/devel/setup.bash 

The code can be launched with the following commands:

roscore

and in another terminal:

roslaunch team-1-robotics team1.launch
