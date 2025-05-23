## Trajectory tracking with WidowX250S arm

For now in this package there is an action client that makes a call to the action server `mobile_wx250s/arm_controller/follow_joint_trajectory` that activates when the arm_controller is launched
```
cd ~/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms
git clone git@github.com:P3dr1x/traj_tracking_py.git
cd ~/interbotix_ws
colcon build --packages-select traj_tracking_py --symlink-install
```

### Example of usage

For executing with the EE the circular trajectory that is contained in the `\trajectories` folder, open another terminal and

```
cd interbotix_ws
. install/setup.bash
ros2 run traj_tracking_py traj_tracker_client ~/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/traj_tracking_py/trajectories/q_traj_circle.csv
```
