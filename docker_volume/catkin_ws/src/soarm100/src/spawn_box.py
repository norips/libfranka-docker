import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

def add_box_to_existing_scene():
    moveit_commander.roscpp_initialize([])
    rospy.init_node("add_box_to_scene", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)  # Wait for scene to initialize

    # Define box pose relative to planning frame
    box_pose = PoseStamped()
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.position.x = 0.51 + 0.485 / 2
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 0.414/2
    box_pose.pose.orientation.w = 1.0

    box_name = "dynamic_box"

    scene.add_box(box_name, box_pose, size=(0.485, 0.6, 0.414))
    rospy.loginfo(f"Added collision box '{box_name}' to the existing planning scene.")

    rospy.spin()

if __name__ == "__main__":
    add_box_to_existing_scene()