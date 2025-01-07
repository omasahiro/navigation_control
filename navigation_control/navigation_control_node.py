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
from datetime import datetime, time

class NavigationControlNode(Node):
    def __init__(self):
        super().__init__('navigation_control_node')
        
        self.get_logger().info('Navigation control node started')
        
        self.callback_group = MutuallyExclusiveCallbackGroup()

        self.resume_service = self.create_service(
            Trigger,
            'resume_navigation',
            self.resume_callback,
            callback_group=self.callback_group
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # 往復区間と時間帯の設定を読み込む
        self.load_schedule()
        self.load_waypoints()

        # 現在の往復区間のインデックス
        self.current_section = None
        self.moving_forward = True  # True: 往路, False: 復路
        self.is_navigating = False

        # 定期的なスケジュールチェック (10秒ごと)
        self.create_timer(10.0, self.check_schedule)
        self.create_timer(30.0, self.reload_schedule)

    def load_schedule(self):
        """スケジュール設定ファイルを読み込む"""
        try:
            config_dir = os.path.join(get_package_share_directory('navigation_control'), 'config')
            schedule_file = os.path.join(config_dir, 'schedule.yaml')
            
            with open(schedule_file, 'r') as f:
                config = yaml.safe_load(f)
                self.schedule = config['schedule']
                self.get_logger().info(f'Loaded {len(self.schedule)} schedule entries')
        except Exception as e:
            self.get_logger().error(f'Failed to load schedule: {str(e)}')
            self.schedule = []

    def load_waypoints(self):
        """往復区間の地点データを読み込む"""
        try:
            config_dir = os.path.join(get_package_share_directory('navigation_control'), 'config')
            sections_file = os.path.join(config_dir, 'sections.yaml')
            
            with open(sections_file, 'r') as f:
                config = yaml.safe_load(f)
                self.sections = config['sections']
                self.get_logger().info(f'Loaded {len(self.sections)} sections')
        except Exception as e:
            self.get_logger().error(f'Failed to load sections: {str(e)}')
            self.sections = []

    def check_schedule(self):
        """現在時刻に基づいて適切な往復区間を選択する"""
        current_time = datetime.now().time()
        new_section = None

        for schedule_entry in self.schedule:
            start_time = datetime.strptime(schedule_entry['start_time'], '%H:%M').time()
            end_time = datetime.strptime(schedule_entry['end_time'], '%H:%M').time()
            
            if self._is_time_between(current_time, start_time, end_time):
                new_section = schedule_entry['section_id']
                break

        if new_section != self.current_section:
            self.current_section = new_section
            self.moving_forward = True
            if not self.is_navigating and self.current_section is not None:
                self.navigate_to_current_goal()

    def reload_schedule(self):
        """設定ファイルを再読み込みする"""
        self.get_logger().info('Reloading schedule...')
        self.load_schedule()
        self.load_waypoints()
    
        # 現在のナビゲーション状態をリセット
        if not self.is_navigating:
            self.current_section = None
            self.moving_forward = True
            self.check_schedule()  # 現在時刻に応じたスケジュールを確認

    def _is_time_between(self, current_time, start_time, end_time):
        """指定された時間が開始時間と終了時間の間にあるかチェック"""
        if start_time <= end_time:
            return start_time <= current_time <= end_time
        else:  # 日をまたぐ場合
            return current_time >= start_time or current_time <= end_time

    def get_current_goal_pose(self):
        """現在の目標地点のPoseを取得"""
        if self.current_section is None or self.current_section not in self.sections:
            return None

        section = self.sections[self.current_section]
        point = section['start'] if self.moving_forward else section['end']

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = point['position']['x']
        goal_pose.pose.position.y = point['position']['y']
        goal_pose.pose.position.z = point['position']['z']
        goal_pose.pose.orientation.x = point['orientation']['x']
        goal_pose.pose.orientation.y = point['orientation']['y']
        goal_pose.pose.orientation.z = point['orientation']['z']
        goal_pose.pose.orientation.w = point['orientation']['w']

        return goal_pose

    def navigate_to_current_goal(self):
        """現在の目標地点へのナビゲーションを開始"""
        if self.current_section is None:
            self.get_logger().info('No active section for current time')
            return False

        goal_pose = self.get_current_goal_pose()
        if goal_pose is None:
            self.get_logger().error('Failed to get goal pose')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        self.get_logger().info(f'Navigating to {"start" if self.moving_forward else "end"} of section {self.current_section}')
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True
        return True

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
            # 往復の向きを切り替え
            self.moving_forward = not self.moving_forward
            # 次の目標地点への移動を開始
            self.navigate_to_current_goal()
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            self.is_navigating = False

    def feedback_callback(self, feedback_msg):
        """ナビゲーションの進捗状況を受け取るコールバック"""
        feedback = feedback_msg.feedback

    async def resume_callback(self, request, response):
        """resumeサービスのコールバック関数"""
        self.get_logger().info('Resume navigation service called')
        
        if self.is_navigating:
            response.success = False
            response.message = 'Navigation is already in progress'
            return response

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