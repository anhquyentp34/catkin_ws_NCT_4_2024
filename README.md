catkin_ws_NCT_4_2024
This research integrates the A* algorithm into the global_planner and the TEB algorithm into the local_planner of move_base. This is accomplished by adding the global_path_planning, srv_client_plugin, and teb_local_planner packages.

Required Packages
To run this workspace, you need to add the following packages:

gazebo_sfm_plugin
lightsfm
teb_local_planner
Running the Program
Step 1: Launch the Experiment
roslaunch diffbot_navigation robot_tranh.launch
Step 2: Run Marker to Indicate Moving Person and Direction
rosrun using_markers marker_publisher5.py
Step 3: Run Marker to Indicate Start and Goal Points of the Robot
rosrun using_markers start_goal_publisher.py
or Star and Goal Marker Publisher
rosrun using_markers start_goal_publisher_with_star_marker.py 

RViz Configuration
Open RViz:
rosrun rviz rviz
Add a Marker display in the Displays panel.
Set the Marker Topic to /visualization_marker.
