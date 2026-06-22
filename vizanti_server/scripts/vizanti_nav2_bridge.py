#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateThroughPoses
from action_msgs.msg import GoalStatus

class VizantiNav2Bridge(Node):
    def __init__(self):
        super().__init__('vizanti_nav2_bridge')

        self.declare_parameter('waypoints_topic', '')
        self.declare_parameter('loop_topic', '')
        self.declare_parameter('action_server_name', '')

        wpt_topic = self.get_parameter('waypoints_topic').get_parameter_value().string_value
        l_topic = self.get_parameter('loop_topic').get_parameter_value().string_value
        act_server = self.get_parameter('action_server_name').get_parameter_value().string_value

        if not wpt_topic or not l_topic or not act_server:
            self.get_logger().fatal("Missing parameters")
            raise SystemExit("Missing parameters")

        self.subscription = self.create_subscription(
            PoseArray,
            wpt_topic,
            self.waypoints_callback,
            10
        )
        
        self.loop_sub = self.create_subscription(
            Bool,
            l_topic,
            self.loop_callback,
            10
        )

        self.action_client = ActionClient(
            self, 
            NavigateThroughPoses, 
            act_server
        )
        
        self.is_looping = False
        self.current_poses = []
        self.current_header = None
        self.active_goal_handle = None

        self.get_logger().info("Vizanti <-> Nav2 Waypoint Bridge initialized.")

    def loop_callback(self, msg: Bool):
        self.is_looping = msg.data
        self.get_logger().info(f"Mission looping set to: {self.is_looping}")

    def waypoints_callback(self, msg: PoseArray):
        # The UI sends an empty array when the user clicks 'Stop'
        if not msg.poses:
            self.get_logger().info("Received empty waypoints. Canceling mission.")
            self.current_poses = []
            if self.active_goal_handle:
                self.active_goal_handle.cancel_goal_async()
                self.active_goal_handle = None
            return

        self.get_logger().info(f"Received {len(msg.poses)} waypoints. Translating to Nav2 Action...")
        self.current_poses = msg.poses
        self.current_header = msg.header
        self.send_mission()

    def send_mission(self):
        if not self.current_poses:
            return

        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Nav2 /navigate_through_poses action server is not available!")
            return

        goal_msg = NavigateThroughPoses.Goal()

        for pose in self.current_poses:
            pose_stamped = PoseStamped()
            pose_stamped.header = self.current_header 
            pose_stamped.pose = pose
            goal_msg.poses.append(pose_stamped)

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.active_goal_handle = future.result()
        if not self.active_goal_handle.accepted:
            self.get_logger().warning("Goal rejected by Nav2.")
            return

        self.get_logger().info("Goal accepted by Nav2, executing...")
        get_result_future = self.active_goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f"Mission finished with status code: {status}")
        
        # If successful, check if we should loop it
        if self.is_looping and self.current_poses and status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Looping enabled. Restarting mission...")
            self.send_mission()
        else:
            self.active_goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    node = VizantiNav2Bridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
