# gps_heading_imu

ROS2 (Jazzy) node that converts a u-blox moving-baseline GPS heading into a standard
`sensor_msgs/Imu`, for use as an absolute-yaw source in `robot_localization`
(`navsat_transform_node` / EKF).

The robot's GPS (ArduSimple simpleRTK2B, dual ZED-F9P, moving-baseline RTK) publishes the
true-north heading of the antenna baseline as `ublox_ubx_msgs/msg/UBXNavRelPosNED` on
`/rover/ubx_nav_rel_pos_ned`. This node republishes it as a yaw-only `sensor_msgs/Imu` on
`/rover/imu`, in the REP-103 ENU convention.

## Conversion

`rel_pos_heading` is an NED compass heading (0 = North, clockwise, degrees × 1e-5). ROS
expects ENU yaw (0 = East, counter-clockwise):

```
yaw_enu = normalize( pi/2 - radians(rel_pos_heading*1e-5 + heading_offset_deg) )
```

Sanity: North → +90°, East → 0°, South → −90°, West → 180°.

Because the heading is already **true-north** referenced, any downstream
`navsat_transform_node` should use `magnetic_declination_radians: 0` and `yaw_offset: 0`.

## Behavior

- Publishes only when the heading is trustworthy: `rel_pos_heading_valid`, `gnss_fix_ok`,
  and `carr_soln.status == 2` (RTK fixed). Each gate is individually toggleable.
- Yaw-only orientation (roll = pitch = 0). Yaw covariance from `acc_heading`
  (`max((acc_heading*1e-5 → rad)², min_yaw_cov)`); roll/pitch set to `unknown_axis_cov`.
- Marks angular velocity and linear acceleration absent (`covariance[0] = -1`, REP
  convention). Copies the source acquisition timestamp.

## Parameters

| Param | Default | Purpose |
|-------|---------|---------|
| `input_topic` | `/rover/ubx_nav_rel_pos_ned` | source UBXNavRelPosNED |
| `output_topic` | `/rover/imu` | IMU output |
| `frame_id` | `base_link` | IMU header frame |
| `heading_offset_deg` | `0.0` | antenna-baseline-to-forward-axis offset (NED/clockwise) |
| `require_heading_valid` | `true` | drop frames with `rel_pos_heading_valid == False` |
| `require_fixed` | `true` | require `carr_soln.status == 2` (RTK fixed) |
| `require_gnss_fix_ok` | `true` | require `gnss_fix_ok == True` |
| `min_yaw_cov` | `(0.5°)²` | yaw covariance floor (rad²) |
| `unknown_axis_cov` | `1e6` | roll/pitch orientation covariance |

## Build & run

```bash
# Requires ublox_ubx_msgs on the AMENT_PREFIX_PATH (it lives in the Pi's sensor_ws).
cd ~/ros2_ws && colcon build --packages-select gps_heading_imu --symlink-install
source install/setup.bash
ros2 run gps_heading_imu gps_heading_imu
# or
ros2 launch gps_heading_imu gps_heading_imu.launch.py heading_offset_deg:=0.0
```

Verify:

```bash
ros2 topic echo /rover/imu --once   # unit quaternion (x≈0,y≈0), orientation_covariance[8]≈accHdg²
ros2 topic hz /rover/imu            # ~5 Hz when carrSoln=fixed
```
