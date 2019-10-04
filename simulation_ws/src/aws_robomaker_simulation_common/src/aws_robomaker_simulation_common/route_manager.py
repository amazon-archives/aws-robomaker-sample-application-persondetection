#!/usr/bin/env python3
"""
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""

import random
import os
import itertools
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion
from ament_index_python.packages import get_package_share_directory


class RouteManager(Node):
    '''Send goals to move_base server for the specified route. Routes forever.
       Loads the route from yaml.
       Use RViz to record 2D nav goals.
       Echo the input goal on topic /move_base_simpl/goal
       Format:
            order: inorder
            poses:
                - pose:
                      position:
                        x: -5.41667556763
                        y: -3.14395284653
                        z: 0.0
                      orientation:
                        x: 0.0
                        y: 0.0
                        z: 0.785181432231
                        w: 0.619265789851
    '''

    # return an iterator over the goals
    route_modes = {
        'inorder': lambda goals: itertools.cycle(goals),
        'random' : lambda goals: (random.choice(goals) for i in itertools.count()),
    }

    def __init__(self):
        super().__init__('route_manager', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        route_file_info = self.get_parameter('route_file').value

        self._action_client = ActionClient(self, NavigateToPose, 'NavigateToPose')
        self._action_client.wait_for_server()
        # time.sleep(10)

        route_pkg_share = get_package_share_directory(route_file_info.split('.')[0])
        route_file_path = os.path.join(route_pkg_share, '.'.join(route_file_info.split('.')[1:]))

        with open(route_file_path, 'r') as f:
            route_file_contents = f.read()
        route_yaml = yaml.safe_load(route_file_contents)

        self.route_mode = route_yaml['mode']
        if self.route_mode not in RouteManager.route_modes:
            self.get_logger().error(
                "Route mode '%s' unknown, exiting route manager" % (self.route_mode, ))
            return

        poses = route_yaml['poses']
        if not poses:
            self.get_logger().info("Route manager initialized no goals, unable to route")

        self.goals = RouteManager.route_modes[self.route_mode](poses)
        self.get_logger().info(
            "Route manager initialized with %s goals in %s mode" % (len(poses), self.route_mode, ))

    def to_move_goal(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position = Point(**pose['pose']['position'])
        goal.pose.pose.orientation = Quaternion(**pose['pose']['orientation'])
        return goal

    def goal_response_callback(self, future):
        self.get_logger().info('goal_response_callback called')
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :( Try again')
            self.send_goal()
            return

        self.get_logger().info('Goal accepted :)')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Feedback received')

    def get_result_callback(self, future):
        self.get_logger().info('Goal reached')
        self.route_forever()

    def send_goal(self):
        send_goal_future = self._action_client.send_goal_async(
            self.current_goal,
            feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def route_forever(self):
        try:
            self.get_logger().info("Route mode is '%s', getting next goal" % (self.route_mode,))
            self.current_goal = self.to_move_goal(next(self.goals))
            self.send_goal()

        except StopIteration:
            self.get_logger().info("No goals, stopping route manager.")
            return

def main(args=None):
    rclpy.init()
    try:
        route_manager = RouteManager()
        route_manager.route_forever()
        rclpy.spin(route_manager)
    except BaseException:
        raise
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
