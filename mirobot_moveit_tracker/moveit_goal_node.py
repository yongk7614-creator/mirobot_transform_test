        )
        
        with self._lock:
            if self._busy:
                self.get_logger().warn("MoveIt is busy. Ignoring new goal.")
                return

            if self.ignore_same_goal and self._is_same_goal(msg, self._last_goal):
                self.get_logger().info("Same goal received. Ignoring.")
                return

            self._busy = True

        thread = threading.Thread(
            target=self._process_goal,
            args=(copy.deepcopy(msg),),
            daemon=True,
        )
        thread.start()

    def _process_goal(self, goal_pose: PoseStamped):
        try:
            self.send_goal_to_moveit(goal_pose)
            self._last_goal = copy.deepcopy(goal_pose)
        except Exception as exc:
            self.get_logger().error("Failed to send goal to MoveIt: %s" % str(exc))
        finally:
            with self._lock:
                self._busy = False

    def send_goal_to_moveit(self, goal_pose: PoseStamped):
        if not goal_pose.header.frame_id:
            raise ValueError("Goal pose has empty frame_id.")

        if not self.accept_any_frame and goal_pose.header.frame_id != self.base_link_name:
            raise ValueError(
                "Goal pose frame_id '%s' does not match expected base frame '%s'."
                % (goal_pose.header.frame_id, self.base_link_name)
            )

        q = goal_pose.pose.orientation
        if self._is_zero_quaternion(q.x, q.y, q.z, q.w):
            raise ValueError("Goal pose has invalid zero quaternion.")

        qx, qy, qz, qw = self._normalize_quaternion(q.x, q.y, q.z, q.w)

        position = [
            float(goal_pose.pose.position.x),
            float(goal_pose.pose.position.y),
            float(goal_pose.pose.position.z),
        ]
        quat_xyzw = [qx, qy, qz, qw]

        self.get_logger().info(
            "Goal validated. Ready to send to MoveIt: pos=%s quat=%s"
            % ([round(v, 4) for v in position], [round(v, 4) for v in quat_xyzw])
        )

        if self.dry_run_only:
            self.get_logger().info(
                "Dry-run only: tracker is ready. Goal would be sent to MoveIt here."
            )
            return

        if self.cartesian:
            self.get_logger().info("Sending cartesian goal to MoveIt.")
            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                cartesian=True,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold,
            )
        else:
            self.get_logger().info("Sending joint-space goal to MoveIt.")
            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                cartesian=False,
            )
            
        if self.execute_motion:
            self.moveit2.wait_until_executed()
            self.get_logger().info("MoveIt goal execution completed.")
        else:
            self.get_logger().info("MoveIt goal was submitted without execution wait.")
            

    def _is_same_goal(self, a: PoseStamped, b: Optional[PoseStamped]) -> bool:
        if b is None:
            return False

        pa = a.pose.position
        pb = b.pose.position
        qa = a.pose.orientation
        qb = b.pose.orientation

        return (
            a.header.frame_id == b.header.frame_id
            and abs(pa.x - pb.x) < 1e-3
            and abs(pa.y - pb.y) < 1e-3
            and abs(pa.z - pb.z) < 1e-3
            and abs(qa.x - qb.x) < 1e-3
            and abs(qa.y - qb.y) < 1e-3
            and abs(qa.z - qb.z) < 1e-3
            and abs(qa.w - qb.w) < 1e-3
        )

    @staticmethod
    def _is_zero_quaternion(x, y, z, w):
        return abs(x) < 1e-12 and abs(y) < 1e-12 and abs(z) < 1e-12 and abs(w) < 1e-12

    @staticmethod
    def _normalize_quaternion(x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-12:
            raise ValueError("Quaternion norm is too small.")
        return x / norm, y / norm, z / norm, w / norm


def main(args=None):
    rclpy.init(args=args)
    node = MoveItGoalNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
