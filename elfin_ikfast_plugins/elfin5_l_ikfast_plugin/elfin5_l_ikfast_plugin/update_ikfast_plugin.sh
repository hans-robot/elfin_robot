search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=elfin5_l.srdf
robot_name_in_srdf=elfin5_l
moveit_config_pkg=elfin5_l_moveit_config
robot_name=elfin5_l
planning_group_name=elfin_arm
ikfast_plugin_pkg=elfin5_l_ikfast_plugin
base_link_name=elfin_base_link
eef_link_name=elfin_end_link
ikfast_output_path=/home/cjw/test_ws/src/elfin_robot/elfin5_l_ikfast_plugin/src/elfin5_l_elfin_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
