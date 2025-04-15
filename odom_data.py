import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import JointState
import tf2_ros
import math

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - \
         math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + \
         math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - \
         math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + \
         math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return (qx, qy, qz, qw)

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        self.wheel_radius = 0.215
        self.wheel_base = 0.80
        self.ticks_per_rev = 360

        self.last_left = 0
        self.last_right = 0
        self.left = 0
        self.right = 0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        self.create_subscription(Int32, 'left_encoder', self.left_callback, 10)
        self.create_subscription(Int32, 'right_encoder', self.right_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.update_odom)  # 20 Hz

    def left_callback(self, msg):
        self.left = msg.data

    def right_callback(self, msg):
        self.right = msg.data

    def update_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9

        d_left = (2 * math.pi * self.wheel_radius * (self.left - self.last_left)) / self.ticks_per_rev
        d_right = (2 * math.pi * self.wheel_radius * (self.right - self.last_right)) / self.ticks_per_rev
        self.last_left = self.left
        self.last_right = self.right

        d_center = (d_left + d_right) / 2.0
        delta_theta = (d_right - d_left) / self.wheel_base

        if delta_theta != 0:
            r = d_center / delta_theta
            icc_x = self.x - r * math.sin(self.th)
            icc_y = self.y + r * math.cos(self.th)
            self.th += delta_theta
            self.x = icc_x + r * math.sin(self.th)
            self.y = icc_y - r * math.cos(self.th)
        else:
            self.x += d_center * math.cos(self.th)
            self.y += d_center * math.sin(self.th)

        quat = euler_to_quaternion(0, 0, self.th)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        odom.twist.twist.linear.x = d_center / dt if dt > 0 else 0.0
        odom.twist.twist.angular.z = delta_theta / dt if dt > 0 else 0.0
        self.odom_pub.publish(odom)

        # TF transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        self.tf_broadcaster.sendTransform(t)

        # JointState for all 4 wheels
        rotation_left = 2 * math.pi * self.left / self.ticks_per_rev
        rotation_right = 2 * math.pi * self.right / self.ticks_per_rev

        js = JointState()
        js.header.stamp = current_time.to_msg()
        js.name = ['Revolute 17', 'Revolute 20', 'Revolute 18', 'Revolute 19']
        js.position = [rotation_left, rotation_right, rotation_left, rotation_right]
        self.joint_pub.publish(js)

        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
