import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft
import numpy as np


class OdometryToMavros:
    def __init__(self):
        rospy.init_node('odom_to_mavros', anonymous=True)

        self.init_flag = False

        self.latest_q_px4 = None

        self.q_align = np.array([0.0, 0.0, 0.0, 1.0])
        self.rot_matrix = np.eye(3)

        self.vision_pub = rospy.Publisher(
            '/mavros/vision_pose/pose', PoseStamped, queue_size=10)

        rospy.Subscriber(
            '/mavros/local_position/odom', Odometry, self.px4_odom_callback)

        rospy.Subscriber(
            '/Odometry', Odometry, self.odom_callback, queue_size=10)

        rospy.loginfo("Waiting for synchronized PX4 & Lidar odometry to initialize FULL rotation alignment...")

    def px4_odom_callback(self, msg):
        self.latest_q_px4 = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

    def odom_callback(self, msg):
        if self.latest_q_px4 is None:
            return

        q_lidar = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        if not self.init_flag:
            q_px4 = self.latest_q_px4

            q_lidar_inv = tft.quaternion_inverse(q_lidar)

            self.q_align = tft.quaternion_multiply(q_px4, q_lidar_inv)

            self.rot_matrix = tft.quaternion_matrix(self.q_align)[:3, :3]

            self.init_flag = True

            rospy.loginfo("FULL ROTATION alignment initialized!")
            return

        p_lidar = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        p_enu = np.dot(self.rot_matrix, p_lidar)

        q_enu = tft.quaternion_multiply(self.q_align, q_lidar)

        vision = PoseStamped()
        vision.header.stamp = msg.header.stamp
        vision.header.frame_id = "map"

        vision.pose.position.x = p_enu[0]
        vision.pose.position.y = p_enu[1]
        vision.pose.position.z = p_enu[2]

        vision.pose.orientation.x = q_enu[0]
        vision.pose.orientation.y = q_enu[1]
        vision.pose.orientation.z = q_enu[2]
        vision.pose.orientation.w = q_enu[3]

        self.vision_pub.publish(vision)


if __name__ == '__main__':
    try:
        OdometryToMavros()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass