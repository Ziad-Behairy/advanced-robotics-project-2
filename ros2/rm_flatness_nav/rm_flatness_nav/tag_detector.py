"""AprilTag detector node (A2 Problem 1).

- Moves the arm to the downward camera pose (SDK X = 105 mm, Y = 30 mm) once at startup.
- Detects the target tag in camera/image_color with pupil-apriltags.
- Publishes the tag center in the base frame (tag/point) plus ground range and bearing (tag/range_bearing).

Only the newest frame is processed (queue depth 1, fixed-rate timer), so a slow detection never
builds a backlog of old frames. This plays the role of the assignment's grab-thread sleep: the
driver owns the H.264 decoder, and this node never holds on to its frame buffers.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from pupil_apriltags import Detector
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robomaster_msgs.action import MoveArm
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

from rm_flatness_nav.geometry import quat_to_rot, range_bearing, transform_point


class TagDetector(Node):

    def __init__(self):
        super().__init__('tag_detector')
        self.target_id = self.declare_parameter('target_id', 3).value
        self.tag_size = self.declare_parameter('tag_size', 0.155).value  # measured black edge (m)
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.rate = self.declare_parameter('rate', 15.0).value
        self.decimate = self.declare_parameter('quad_decimate', 1.0).value
        self.init_arm = self.declare_parameter('init_arm', True).value
        self.arm_x = self.declare_parameter('arm_x', 0.105).value  # m, SDK X
        self.arm_z = self.declare_parameter('arm_z', 0.030).value  # m, SDK Y (up)

        self.detector = Detector(families='tag36h11', quad_decimate=self.decimate, refine_edges=True)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_params = None   # (fx, fy, cx, cy)
        self.latest_image = None
        self.n_frames = 0
        self.n_hits = 0

        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info, 1)
        self.create_subscription(Image, 'camera/image_color', self.on_image, qos_profile_sensor_data)
        self.point_pub = self.create_publisher(PointStamped, 'tag/point', 10)
        self.rb_pub = self.create_publisher(Float32MultiArray, 'tag/range_bearing', 10)
        self.debug_pub = self.create_publisher(Image, 'tag/debug_image', 1)
        self.create_timer(1.0 / self.rate, self.process)
        self.create_timer(5.0, self.report)

        if self.init_arm:
            self.arm_client = ActionClient(self, MoveArm, 'move_arm')
            self.arm_timer = self.create_timer(1.0, self.send_arm_goal)

    def send_arm_goal(self):
        if not self.arm_client.server_is_ready():
            self.get_logger().info('Waiting for move_arm action server...', throttle_duration_sec=5.0)
            return
        self.arm_timer.cancel()
        goal = MoveArm.Goal(x=self.arm_x, z=self.arm_z, relative=False)
        self.arm_client.send_goal_async(goal)
        self.get_logger().info(f'Arm init goal sent: x={self.arm_x:.3f} m, z={self.arm_z:.3f} m')

    def on_info(self, msg):
        if self.camera_params is None:
            k = msg.k
            self.camera_params = (k[0], k[4], k[2], k[5])
            self.get_logger().info(f'Camera intrinsics {msg.width}x{msg.height}: '
                                   f'fx={k[0]:.1f} fy={k[4]:.1f} cx={k[2]:.1f} cy={k[5]:.1f}')

    def on_image(self, msg):
        self.latest_image = msg

    def process(self):
        msg, self.latest_image = self.latest_image, None
        if msg is None or self.camera_params is None:
            return
        self.n_frames += 1
        gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        detections = self.detector.detect(gray, estimate_tag_pose=True,
                                          camera_params=self.camera_params, tag_size=self.tag_size)
        target = next((d for d in detections if d.tag_id == self.target_id), None)

        if self.debug_pub.get_subscription_count() > 0:
            self.publish_debug(msg, gray, detections)
        if target is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'No TF {msg.header.frame_id} -> {self.base_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return
        q, t = tf.transform.rotation, tf.transform.translation
        p_base = transform_point(quat_to_rot(q.x, q.y, q.z, q.w), (t.x, t.y, t.z), target.pose_t.ravel())
        d, bearing = range_bearing(p_base)
        self.n_hits += 1

        pt = PointStamped()
        pt.header.stamp = msg.header.stamp
        pt.header.frame_id = self.base_frame
        pt.point.x, pt.point.y, pt.point.z = (float(v) for v in p_base)
        self.point_pub.publish(pt)
        # [d, bearing, decision_margin, image_stamp_sec]: the stamp lets the planner measure tag age
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.rb_pub.publish(Float32MultiArray(data=[d, bearing, float(target.decision_margin), stamp]))

    def publish_debug(self, msg, gray, detections):
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for det in detections:
            color = (0, 200, 0) if det.tag_id == self.target_id else (0, 160, 255)
            cv2.polylines(img, [det.corners.astype(np.int32)], True, color, 2)
            cx, cy = det.center.astype(int)
            cv2.putText(img, f'T{det.tag_id}', (cx - 15, cy + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        h, w = img.shape[:2]
        cv2.line(img, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        out = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        out.header = msg.header
        self.debug_pub.publish(out)

    def report(self):
        self.get_logger().info(f'{self.n_frames / 5.0:.1f} frames/s processed, '
                               f'T{self.target_id} seen in {self.n_hits}')
        self.n_frames = self.n_hits = 0


def main():
    rclpy.init()
    node = TagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
