roslaunch franka_gazebo panda.launch x:=-0.5 z:=1 \
    world:=empty.srdf \
    controller:=cartesian_impedance_example_controller \
    interactive_marker:=true \
    rviz:=true
