#!/usr/bin/env python3.8
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped

class PoseToTwist:
    def __init__(self):
        # Params
        self.base_frame = rospy.get_param("~base_frame", "panda_link0")
        self.ee_frame   = rospy.get_param("~ee_frame", "panda_link8")  # change to your EE link
        self.linear_gain = rospy.get_param("~linear_gain", 10.0)         # m/s per meter error
        self.angular_gain = rospy.get_param("~angular_gain", 10.0)       # rad/s per rad error
        self.rate_hz = rospy.get_param("~rate", 100)

        # ROS interfaces
        self.listener = tf.TransformListener()
        self.target_pose = None
        self.target_sub = rospy.Subscriber("/target_pose", PoseStamped, self.target_cb)
        self.target_sub_debug = rospy.Publisher("/target_pose_debug", PoseStamped)
        self.cmd_pub = rospy.Publisher("/servo_server/delta_twist_cmds",
                                       TwistStamped, queue_size=1)

        rospy.loginfo("pose_to_twist node started. Waiting for /target_pose ...")
        self.run()

    def target_cb(self, msg):
        # Assume target_pose is expressed in base_frame
        self.target_pose = msg

    def get_current_pose(self):
        try:
            self.listener.waitForTransform(self.base_frame, self.ee_frame,
                                           rospy.Time(0), rospy.Duration(0.1))
            (trans, rot) = self.listener.lookupTransform(self.base_frame,
                                                         self.ee_frame,
                                                         rospy.Time(0))
            return np.array(trans), np.array(rot)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            return None, None

    def quaternion_multiply(self, q1, q2):
        # xyzw order
        w0, x0, y0, z0 = q1[3], q1[0], q1[1], q1[2]
        w1, x1, y1, z1 = q2[3], q2[0], q2[1], q2[2]
        return np.array([
            x0 * w1 + w0 * x1 + y0 * z1 - z0 * y1,
            y0 * w1 + w0 * y1 + z0 * x1 - x0 * z1,
            z0 * w1 + w0 * z1 + x0 * y1 - y0 * x1,
            w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1])

    def quaternion_inverse(self, q):
        q_conj = np.array([-q[0], -q[1], -q[2], q[3]])
        return q_conj / np.dot(q, q)

    def quaternion_to_axis_angle(self, q):
        """Convert quaternion (xyzw) to axis-angle vector."""
        q = q / np.linalg.norm(q)
        angle = 2 * np.arccos(q[3])
        if angle < 1e-6:
            return np.zeros(3)
        axis = q[:3] / np.sin(angle / 2.0)
        return axis * angle

    def run(self):
        rospy.loginfo("pose_to_twist node running")
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.target_pose is None:
                rate.sleep()
                continue

            current_pos, current_quat = self.get_current_pose()
            if current_pos is None:
                rate.sleep()
                continue

            pose = PoseStamped()
            pose.header.frame_id = "panda_link0"
            pose.pose.position.x = current_pos[0]
            pose.pose.position.y = current_pos[1]
            pose.pose.position.z = current_pos[2]
            pose.pose.orientation.x = current_quat[0]
            pose.pose.orientation.y = current_quat[1]
            pose.pose.orientation.z = current_quat[2]
            pose.pose.orientation.w = current_quat[3]
            self.target_sub_debug.publish(pose)

            # Target
            tgt_pos = np.array([
                self.target_pose.pose.position.x,
                self.target_pose.pose.position.y,
                self.target_pose.pose.position.z])
            tgt_quat = np.array([
                self.target_pose.pose.orientation.x,
                self.target_pose.pose.orientation.y,
                self.target_pose.pose.orientation.z,
                self.target_pose.pose.orientation.w])

            # Compute errors
            pos_error = tgt_pos - current_pos
            # orientation error: q_err = q_current^-1 * q_target
            q_err = self.quaternion_multiply(self.quaternion_inverse(current_quat), tgt_quat)
            ang_error = self.quaternion_to_axis_angle(q_err)

            # Compose Twist
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.header.frame_id = self.base_frame
            twist.twist.linear.x = self.linear_gain * pos_error[0]
            twist.twist.linear.y = self.linear_gain * pos_error[1]
            twist.twist.linear.z = self.linear_gain * pos_error[2]
            twist.twist.angular.x = self.angular_gain * ang_error[0]
            twist.twist.angular.y = self.angular_gain * ang_error[1]
            twist.twist.angular.z = self.angular_gain * ang_error[2]

            self.cmd_pub.publish(twist)
            rospy.loginfo(f"pose_to_twist node published {twist}")
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("pose_to_twist")
    PoseToTwist()
