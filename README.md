### For Success:

Start a fresh simulation with:

    source ~/ros2_ws/install/setup.bash
    ros2 launch fastbot_gazebo one_fastbot_room.launch.py

Then, in ~/ros2_ws/src/fastbot_action_server, make sure lines 85-91 are commented. 
These make the server return success immediately without actually performing the movements. 

Then, in a second terminal, launch the fastbot action server with:

    cd ~/ros2_ws && colcon build && source install/setup.bash
    ros2 run fastbot_waypoints fastbot_action_server

And, in a third terminal, to run the tests, run:

    cd ~/ros2_ws && colcon build && source install/setup.bash
    colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
    colcon test-result --all

The final command should generate similar to the following output:

    build/fastbot_waypoints/Testing/20261102-2200/Test.xml: 1 test, 0 errors, 0 failures, 0 skipped
    build/fastbot_waypoints/test_results/fastbot_waypoints/fastbot_waypoints_test.gtest.xml: 1 test, 0 errors, 0 failures, 0 skipped

    Summary: 2 tests, 0 errors, 0 failures, 0 skipped

### For Failure:

Stop all running simulations, tests, and servers.

Then, start a fresh simulation with:

    source ~/ros2_ws/install/setup.bash
    ros2 launch fastbot_gazebo one_fastbot_room.launch.py

Then, in ~/ros2_ws/src/fastbot_action_server, make sure the aforementioned lines 85-91 are uncommented (the top comment still remains a comment!). 

Then, in a second terminal, build and launch the fastbot action server with:

    cd ~/ros2_ws && colcon build && source install/setup.bash
    ros2 run fastbot_waypoints fastbot_action_server

You can ignore the build warnings if they show up.
And, in a third terminal, to run the tests, run:

    cd ~/ros2_ws && colcon build && source install/setup.bash
    colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
    colcon test-result --all

The final command should generate similar to the following output:

    build/fastbot_waypoints/Testing/20261102-2204/Test.xml: 1 test, 0 errors, 1 failure, 0 skipped
    build/fastbot_waypoints/test_results/fastbot_waypoints/fastbot_waypoints_test.gtest.xml: 1 test, 0 errors, 1 failure, 0 skipped

    Summary: 2 tests, 0 errors, 2 failures, 0 skipped