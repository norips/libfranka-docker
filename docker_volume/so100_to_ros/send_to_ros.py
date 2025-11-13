import zmq
import rospy
from geometry_msgs.msg import PoseStamped
from franka_gripper.msg import GraspActionGoal, MoveActionGoal
import json
import math

import numpy as np

# ZMQ subscriber setup
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5555")  # replace with container IP if needed
socket.setsockopt_string(zmq.SUBSCRIBE, "")

rospy.init_node("ee_pose_commander")
pub = rospy.Publisher("/cartesian_impedance_example_controller/equilibrium_pose", PoseStamped, queue_size=10)
pub_gripper = rospy.Publisher("/franka_gripper/grasp/goal", GraspActionGoal, queue_size=10)
pub_gripper_move = rospy.Publisher("/franka_gripper/move/goal", MoveActionGoal, queue_size=10)
pub2 = rospy.Publisher("ee_pose_commander", PoseStamped, queue_size=10)

rate = rospy.Rate(100)

scale_ratio = 1.8
prev_gripper = None

bins = np.linspace(0, 1.0, 10)
print(bins)
while not rospy.is_shutdown():
    try:
        is_diff = False

        # dict_keys(['ee.x', 'ee.y', 'ee.z', 'ee.wx', 'ee.wy', 'ee.wz', 'ee.gripper_pos'])
        msg = socket.recv_string(flags=zmq.NOBLOCK)

        data = json.loads(msg)
        pose = PoseStamped()
        pose.header.frame_id = "panda_link0"
        pose.pose.position.x = data["ee.x"] * scale_ratio
        pose.pose.position.y = data["ee.y"] * scale_ratio
        pose.pose.position.z = data["ee.z"] * scale_ratio
        pose.pose.orientation.x = data["ee.qx"] 
        pose.pose.orientation.y = data["ee.qy"] 
        pose.pose.orientation.z = data["ee.qz"]   
        pose.pose.orientation.w = data["ee.qw"]

        goal_gripper = GraspActionGoal()
        goal_gripper.goal.width = 0.01 + 0.07 * bins[np.digitize((data["ee.gripper_pos"] / 100.0), bins, right=True)-1]
        goal_gripper.goal.epsilon.inner = 0.005
        goal_gripper.goal.epsilon.outer = 0.005
        goal_gripper.goal.speed = 0.1
        goal_gripper.goal.force = 5.0

        move_gripper = MoveActionGoal()
        move_gripper.goal.width = 0.001 + 0.07 * bins[np.digitize((data["ee.gripper_pos"] / 100.0), bins, right=False)-1]
        move_gripper.goal.speed = 0.1

        pub_gripper_move.publish(move_gripper)
        # pub_gripper.publish(goal_gripper)

        # if prev_gripper is None:
        #     prev_gripper = data["ee.gripper_pos"]
        # if abs(prev_gripper - data["ee.gripper_pos"]) > 5:
        #     pub_gripper_move.publish(move_gripper)
        #     # pub_gripper.publish(goal_gripper)
        #     prev_gripper = data["ee.gripper_pos"]
        #     is_diff = True

        pub2.publish(pose)
        pub.publish(pose)


        debug = {
            "qx": data["ee.qx"],
            "qy": data["ee.qy"],
            "qz": data["ee.qz"],
            "qw": data["ee.qw"],
            "gripper": data["ee.gripper_pos"] / 100.0
        }
        rospy.loginfo(f"Received from Lerobot: {debug}")
    except zmq.Again:
        pass
    rate.sleep()