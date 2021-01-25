### MoveIt! RViz Plugin Tutorial

More detailed tutorial:  
http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/ros_visualization/visualization_tutorial.html

#### Set start state

In the “Planning” tab of the “Motion Planning” subwindow, set the "Select Start State" field to “current”. Then click the "Update" button.

![image1](images/set_start_state.png)

#### Set goal state

Use the marker (which are attached to the tip link of the arm) to drag the arm to a desired goal state.

![image2](images/set_goal_state.png)

#### Plan a trajectory

After setting the start state and goal state, wait for around 5 seconds, then press the "Plan" button. You should be able to see a visualization of the arm moving.

![image3](images/trajectory.png)

#### Execute the trajectory

Press the "Execute" button, robot will move as planned. In this process, you can press the "Stop" button to make the robot stop.

![image4](images/execution.png)