roslaunch franka_gazebo panda.launch z:=0.717 yaw:=-1.5707 \
    world:=/docker_volume/gazebo_model/demo_world_no_panda.sdf \
    controller:=cartesian_impedance_example_controller \
    interactive_marker:=false \
    rviz:=true
