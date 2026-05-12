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
            "pose_topic":             "/aruco_tf",
            "wheel_status_topic":     "/wheel_status",
            "goal_topic":             "/mirobot_goal_pose",
            "use_tf_transform":       False,
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
           
            "remap_axes":             True,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)

        self.latest_pose = None
        self.prev_is_stopped = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_sub = self.create_subscription(
            PoseArray, self.pose_topic, self.pose_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, self.wheel_status_topic, self.status_callback, 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        self.get_logger().info(
            "WheelStopToGoalNode ready. "
            "pose_topic=%s  use_tf_transform=%s  goal_frame=%s  remap_axes=%s"
            % (self.pose_topic, self.use_tf_transform, self.goal_frame, self.remap_axes)
        )


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
            self.get_logger().warn(
                "[TF] LookupException  %s -> %s : %s"
                % (src_frame, self.goal_frame, e)
            )
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(
                "[TF] ConnectivityException  %s -> %s : %s"
                % (src_frame, self.goal_frame, e)
            )
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(
                "[TF] ExtrapolationException  %s -> %s : %s"
                % (src_frame, self.goal_frame, e)
            )
        return None


    def pose_callback(self, msg):
        """
        Always update latest_pose whenever aruco_tf is received.
        stopped 직전의 최신 좌표를 항상 보관한다.
        """
        if not msg.poses:
            return

        raw = PoseStamped()
        raw.header = copy.deepcopy(msg.header)
        raw.pose = copy.deepcopy(msg.poses[0])

        if self.use_tf_transform:
            transformed = self._transform_to_goal_frame(copy.deepcopy(raw))
        else:
            transformed = copy.deepcopy(raw)

        if transformed is None:
            self.get_logger().warn("TF transform failed.")
            return

        self.latest_pose = copy.deepcopy(transformed)

    def status_callback(self, msg):
        """
        stopped 수신 즉시 latest_pose 를 발행한다.
        """
        is_stopped = msg.data.strip().lower() == "stopped"

        if not is_stopped:
            self.prev_is_stopped = False
            return

        if self.prev_is_stopped:
            return

        if self.latest_pose is None:
            self.get_logger().warn(
                "No ArUco pose received yet. "
                "Check that aruco_tf is being published before stopped."
            )
            return

        self.prev_is_stopped = True
        self.get_logger().info(
            "Wheel stopped. Publishing latest ArUco pose immediately."
        )
        self.publish_goal()


    def publish_goal(self):
        """
        축 재매핑(remap_axes) 후 offset 을 MoveIt 기준으로 적용하여 발행.

        처리 순서:
          1. 축 재매핑 (remap_axes=True 일 때)
               카메라 y  →  MoveIt x
               카메라 z  →  MoveIt y
               카메라 x  →  MoveIt z (부호 반전)
          2. offset 적용 (MoveIt base_link 기준)
               x += offset_x
               y += offset_y
               z += offset_z

        offset 을 재매핑 이후에 더하므로
        offset_x/y/z 가 항상 MoveIt(base_link) 축 기준으로 동작한다.
        """
        goal_pose = copy.deepcopy(self.latest_pose)
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.goal_frame

        ########################################################
        # [1단계] 축 재매핑
        if self.remap_axes:
            cam_x = goal_pose.pose.position.x
            cam_y = goal_pose.pose.position.y
            cam_z = goal_pose.pose.position.z

            moveit_x = cam_z
            moveit_y = -cam_x
            moveit_z = -cam_y
            
            goal_pose.pose.position.x =  moveit_x   
            goal_pose.pose.position.y =  moveit_y   
            goal_pose.pose.position.z = moveit_z  
        ##########################################################
            self.get_logger().info(
                "[remap_axes] "
                "cam(x=%.4f y=%.4f z=%.4f) → moveit(x=%.4f y=%.4f z=%.4f)"
                % (
                    cam_x, cam_y, cam_z,
                    goal_pose.pose.position.x,
                    goal_pose.pose.position.y,
                    goal_pose.pose.position.z,
                )
            )

        # [2단계] offset 적용 (MoveIt base_link 기준)
        goal_pose.pose.position.x += self.offset_x
        goal_pose.pose.position.y += self.offset_y
        goal_pose.pose.position.z += self.offset_z

        # orientation 처리
        if not self.use_marker_orientation:
            goal_pose.pose.orientation.x = self.goal_qx
            goal_pose.pose.orientation.y = self.goal_qy
            goal_pose.pose.orientation.z = self.goal_qz
            goal_pose.pose.orientation.w = self.goal_qw

        self.goal_pub.publish(goal_pose)
        self.get_logger().info(
            "Goal published (in %s): x=%.4f y=%.4f z=%.4f  "
            "qx=%.4f qy=%.4f qz=%.4f qw=%.4f"
            % (
                goal_pose.header.frame_id,
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
                goal_pose.pose.position.z,
                goal_pose.pose.orientation.x,
                goal_pose.pose.orientation.y,
                goal_pose.pose.orientation.z,
                goal_pose.pose.orientation.w,
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
