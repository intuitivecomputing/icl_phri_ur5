# README
## Tutorial:
### Bring up the robot:
`roslaunch ur5_gripper_bringup ur5_bringup.launch with_gripper:=true limited:=true robot_ip:=192.168.1.129`

### Launch moveit scence:
`roslaunch ur5_gripper_moveit_config ur5_gripper_moveit_planning_execution.launch limited:=true`
`roslaunch ur5_gripper_moveit_config moveit_rviz.launch config:=true`

### Launch gripper node
`rosrun robotiq_c_model_control CModelRtuNode.py /dev/ttyUSB1`
`rosrun robotiq_c_model_control CModelSimpleController.py`
`rosrun robotiq_c_model_control CModelStatusListener.py`

### Launch force torque sensor
`rosrun robotiq_force_torque_sensor rq_sensor _serial_id:="ttyUSB0"`

## Bring up all at once
`roslaunch ur5_gripper_bringup ur5_with_gripper.launch`
