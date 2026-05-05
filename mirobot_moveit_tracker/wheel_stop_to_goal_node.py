import copy

import rclpy
import rclpy.duration
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

import tf2_ros
import tf2_geometry_msgs


class WheelStopToGoalNode(Node):
    def __init__(self):
        super().__init__("wheel_stop_to_goal_node")

        defaults = {
            "pose_topic":             "/aruco_poses",
            "wheel_status_topic":     "/wheel_status",
            "goal_topic":             "/mirobot_goal_pose",
            "sample_delay_sec":       0.2,
            "sample_count":           5,
            "offset_x":               0.0,
            "offset_y":               0.0,
            "offset_z":               0.0,
            "goal_frame":             "base_link",
            "use_marker_orientation": True,
            "goal_qx":                0.0,
            "goal_qy":                0.0,
            "goal_qz":                0.0,
            "goal_qw":                1.0,
            "tf_timeout_sec":         0.5,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)

        self.latest_pose = None
        self.prev_is_stopped = False
        self.collecting = False
        self.sample_buffer = []        
        self.delay_timer = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_sub = self.create_subscription(
            PoseArray, self.pose_topic, self.pose_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, self.wheel_status_topic, self.status_callback, 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

    def reset_sampling(self):
        self.collecting = False
        self.sample_buffer = []
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.delay_timer = None

    def _transform_to_goal_frame(self, pose_stamped):  
        src_frame = pose_stamped.header.frame_id
        try:
            transformed = self.tf_buffer.transform(
                pose_stamped,
                self.goal_frame,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
            )
            return transformed
        except tf2_ros.LookupException as e:
            self.get_logger().warn("[TF] LookupException  %s -> %s : %s" % (src_frame, self.goal_frame, e))
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn("[TF] ConnectivityException  %s -> %s : %s" % (src_frame, self.goal_frame, e))
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn("[TF] ExtrapolationException  %s -> %s : %s" % (src_frame, self.goal_frame, e))
        return None

    def pose_callback(self, msg):
        if not msg.poses:
            return

        raw = PoseStamped()
        raw.header = copy.deepcopy(msg.header)
        raw.pose = copy.deepcopy(msg.poses[0])

        transformed = self._transform_to_goal_frame(copy.deepcopy(raw))

        if transformed is None:
            self.get_logger().warn(
                "TF transform failed." 
            )
            return

        self.latest_pose = copy.deepcopy(transformed)

        if not self.collecting:
            return

        self.sample_buffer.append(copy.deepcopy(transformed))

        if len(self.sample_buffer) >= self.sample_count:
            self.publish_averaged_goal()
            self.reset_sampling()

    def status_callback(self, msg):
        is_stopped = msg.data.strip().lower() == "stopped"

        if not is_stopped:
            self.prev_is_stopped = False
            self.reset_sampling()
            return

        if self.prev_is_stopped:
            return

        if self.latest_pose is None:
            self.get_logger().warn("No ArUco pose received yet.")
            return

        self.prev_is_stopped = True
        self.reset_sampling()
        self.delay_timer = self.create_timer(self.sample_delay_sec, self.start_sampling_once)

        self.get_logger().info(
            "Wheel stopped. Waiting %.3f sec before collecting %d samples."
            % (self.sample_delay_sec, self.sample_count)
        )

    def start_sampling_once(self):
        self.reset_sampling()
        self.collecting = True
        self.get_logger().info("Started ArUco pose sampling.")

    def publish_averaged_goal(self):
        if not self.sample_buffer:
            self.get_logger().warn("No samples collected.")
            return

        n = len(self.sample_buffer)
        avg_x = sum(p.pose.position.x for p in self.sample_buffer) / n
        avg_y = sum(p.pose.position.y for p in self.sample_buffer) / n
        avg_z = sum(p.pose.position.z for p in self.sample_buffer) / n

        goal_pose = copy.deepcopy(self.sample_buffer[-1])
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.goal_frame
        goal_pose.pose.position.x = avg_x + self.offset_x
        goal_pose.pose.position.y = avg_y + self.offset_y
        goal_pose.pose.position.z = avg_z + self.offset_z

        if not self.use_marker_orientation:
            goal_pose.pose.orientation.x = self.goal_qx
            goal_pose.pose.orientation.y = self.goal_qy
            goal_pose.pose.orientation.z = self.goal_qz
            goal_pose.pose.orientation.w = self.goal_qw

        self.goal_pub.publish(goal_pose)  
        self.get_logger().info(
            "Averaged pose published (in %s): x=%.4f y=%.4f z=%.4f"
            % (
                self.goal_frame,
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
                goal_pose.pose.position.z,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = WheelStopToGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
