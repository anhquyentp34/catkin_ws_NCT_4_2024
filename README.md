# catkin_ws_NCT_4_2024
Đây là nghiên cứu đưa thuật tóan A* vào global_planner và thuật toán TEB vào local_planner của move_base
thực hiện việc này bằng cách thêm gói
global_path_planning, 
srv_client_plugin,
teb_local_planer


Để chay được không gian nay cần thêm 3 gói sau:

https://github.com/robotics-upo/gazebo_sfm_plugin
https://github.com/robotics-upo/lightsfm
https://github.com/rst-tu-dortmund/teb_local_planner
Chay lại chương trình:
Bước 1: chay chương trình thí nghiêm
roslaunch diffbot_navigation robot_tranh.launch 
Bước 2: chạy marker để đánh dấu người di chuyển và hướng di chuyển(chay trong terminal khác)
rosrun using_markers marker_publisher5.py
Bước 3: Chay marker để đánh dấu điểm xuất phát và điểm đích của robot và người(chạy trong terminal khác)
rosrun using_markers start_goal_publisher.py 
