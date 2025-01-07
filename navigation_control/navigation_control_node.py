#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class NavigationControlNode(Node):
    def __init__(self):
        super().__init__('navigation_control_node')
        
        # ROSロガーの設定
        self.get_logger().info('Navigation control node started')
        
        # コールバックグループの設定
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # resumeサービスの設定
        self.resume_service = self.create_service(
            Trigger,
            'resume_navigation',
            self.resume_callback,
            callback_group=self.callback_group
        )

        # Nav2アクションクライアントの設定
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # 目標地点の読み込み
        self.load_waypoints()

        # 現在の目標地点のインデックス
        self.current_goal_index = 0
        
        # ナビゲーション状態
        self.is_navigating = False

    def load_waypoints(self):
        """設定ファイルから目標地点を読み込む"""
        try:
            config_dir = os.path.join(get_package_share_directory('navigation_control'), 'config')
            waypoints_file = os.path.join(config_dir, 'waypoints.yaml')
            
            with open(waypoints_file, 'r') as f:
                config = yaml.safe_load(f)
                self.waypoints = config['waypoints']
                self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {str(e)}')
            self.waypoints = []

    def goal_response_callback(self, future):
        """ナビゲーションゴールが受け付けられたときのコールバック"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """ナビゲーション結果を受け取ったときのコールバック"""
        result = future.result().result
        status = future.result().status
        if status == 4:  # 成功した場合
            self.get_logger().info('Navigation succeeded')
            # 次の目標地点へ更新
            self.current_goal_index = (self.current_goal_index + 1) % len(self.waypoints)
            # 次の目標地点への移動を開始
            self.navigate_to_current_goal()
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            self.is_navigating = False

    def navigate_to_current_goal(self):
        """現在の目標地点へのナビゲーションを開始"""
        if not self.waypoints:
            self.get_logger().error('No waypoints available')
            return False

        current_goal = self.waypoints[self.current_goal_index]
        
        # PoseStampedメッセージの作成
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = current_goal['position']['x']
        goal_pose.pose.position.y = current_goal['position']['y']
        goal_pose.pose.position.z = current_goal['position']['z']
        goal_pose.pose.orientation.x = current_goal['orientation']['x']
        goal_pose.pose.orientation.y = current_goal['orientation']['y']
        goal_pose.pose.orientation.z = current_goal['orientation']['z']
        goal_pose.pose.orientation.w = current_goal['orientation']['w']

        # ナビゲーションゴールの作成
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Nav2クライアントの準備確認
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # ナビゲーション開始
        self.get_logger().info(f'Navigating to goal {self.current_goal_index}')
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True
        return True

    def feedback_callback(self, feedback_msg):
        """ナビゲーションの進捗状況を受け取るコールバック"""
        feedback = feedback_msg.feedback
        # 必要に応じてフィードバック情報を処理

    async def resume_callback(self, request, response):
        """resumeサービスのコールバック関数"""
        self.get_logger().info('Resume navigation service called')
        
        # すでにナビゲーション中の場合は新しいリクエストを拒否
        if self.is_navigating:
            response.success = False
            response.message = 'Navigation is already in progress'
            return response

        # ナビゲーション開始
        if self.navigate_to_current_goal():
            response.success = True
            response.message = 'Navigation started successfully'
        else:
            response.success = False
            response.message = 'Failed to start navigation'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = NavigationControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()