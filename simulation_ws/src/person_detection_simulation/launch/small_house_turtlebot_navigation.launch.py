# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

'''
A small house with a Turtlebot navigating to pre-determined
goals in a random order endlessly.
Note that navigation nodes are in the simulation application
as it uses a virtual map and should not be deployed to
the real robot.
'''
import os
import sys

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

def generate_launch_description():

    ########################
    ##  Evalution Worker  ##
    ########################
    person_detection_dir = get_package_share_directory('person_detection_simulation')
    person_detection_launch_dir = os.path.join(person_detection_dir, 'launch')

    #############################
    ##  follow_route Argument  ##
    #############################
    follow_route = launch.actions.DeclareLaunchArgument(
        'follow_route',
        default_value='true',
        description='Argument for follow pre-defined route forever')

    gui = launch.actions.DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Argument for display on GUI')

    ###########################
    ##  Create World Launch  ##
    ###########################
    aws_robomaker_small_house_world_dir = get_package_share_directory(
        'aws_robomaker_small_house_world')
    small_house_turtlebot_navigation_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                aws_robomaker_small_house_world_dir,
                'launch',
                'small_house_turtlebot_navigation.launch.py')))

    ###################################
    ##          Route Node           ##
    ###################################
    route_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(person_detection_launch_dir, 'route_manager.launch.py')))

    ##############################################
    ##  H264 Video Encoding and Kinesis Nodes   ##
    ##############################################
    kinesis_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(person_detection_launch_dir, 'kinesis.launch.py')))

    monitoring_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(person_detection_launch_dir, 'monitoring.launch.py')))

    ########################
    ##  Launch Evalution  ##
    ########################
    ld = launch.LaunchDescription([
        gui,
        follow_route,
        small_house_turtlebot_navigation_launch,
        kinesis_node,
        monitoring_node,
        route_manager
        ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
