#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Bool

# The Official Nav2 API
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class VizantiMissionExecutive(Node):
    def __init__(self):
        super().__init__('vizanti_nav2_bridge')
        
        self.navigator = BasicNavigator(node_name='vizanti_basic_navigator')
        
        # Listen to Vizanti UI (Isolated Topic)
        self.subscription = self.create_subscription(PoseArray, '/vizanti_waypoints', self.waypoints_callback, 10)
        self.loop_sub = self.create_subscription(Bool, '/waypoints_loop', self.loop_callback, 10)
        
        self.is_looping = False
        self.current_poses = []
        self.mission_active = False

        # Check Nav2 status every 0.5s without blocking ROS callbacks
        self.timer = self.create_timer(0.5, self.monitor_mission)

        self.get_logger().info("Mission Executive initialized via nav2_simple_commander.")

    def loop_callback(self, msg: Bool):
        self.is_looping = msg.data

    def waypoints_callback(self, msg: PoseArray):
        if not msg.poses:
            self.get_logger().info("Empty waypoints received. Canceling task...")
            self.navigator.cancelTask()
            self.current_poses = []
            self.mission_active = False
            return

        self.get_logger().info(f"Received {len(msg.poses)} waypoints. Generating plan...")
        self.current_poses = []
        for pose in msg.poses:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = pose
            self.current_poses.append(pose_stamped)

        self.start_mission()

    def start_mission(self):
        if not self.current_poses: return
        
        self.get_logger().info("Dispatching waypoints to Nav2...")
        self.navigator.followWaypoints(self.current_poses)
        self.mission_active = True

    def monitor_mission(self):
        if not self.mission_active:
            return

        # Let the API check the internal state machine
        if not self.navigator.isTaskComplete():
            return

        self.mission_active = False
        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Waypoints successfully reached!")
            
            if self.is_looping and len(self.current_poses) > 1:
                self.get_logger().info("Patrol loop enabled. Reversing route...")
                self.current_poses.reverse()
                
                # Prevent tf2 extrapolation by refreshing timestamp
                now = self.navigator.get_clock().now().to_msg()
                for p in self.current_poses:
                    p.header.stamp = now
                    
                self.start_mission()
                
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Mission canceled by user.")
        elif result == TaskResult.FAILED:
            self.get_logger().error("Nav2 failed to complete mission.")

def main(args=None):
    rclpy.init(args=args)
    executive = VizantiMissionExecutive()
    
    # We use a MultiThreadedExecutor so the Navigator API and our Node don't block each other
    executor = MultiThreadedExecutor()
    executor.add_node(executive)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executive.navigator.lifecycleShutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
