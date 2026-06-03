#!/usr/bin/env python3
"""Convert ublox moving-baseline GPS heading into a standard sensor_msgs/Imu.

The robot's GPS (ArduSimple simpleRTK2B, dual ZED-F9P, moving-baseline RTK) publishes
the true-north heading of the antenna baseline in ublox_ubx_msgs/UBXNavRelPosNED on
/rover/ubx_nav_rel_pos_ned. robot_localization (navsat_transform_node / EKF) wants that
heading as a sensor_msgs/Imu orientation in the REP-103 ENU convention.

NED compass heading (0 = North, clockwise) -> ENU yaw (0 = East, counter-clockwise):

    yaw_enu = pi/2 - radians(heading_deg + heading_offset_deg)   # normalized to (-pi, pi]

Because the GPS heading is already referenced to TRUE north, any downstream
navsat_transform should use magnetic_declination_radians = 0 and yaw_offset = 0.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from ublox_ubx_msgs.msg import CarrSoln, UBXNavRelPosNED
import rclpy.qos

# CarrSoln.status == 2 (CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES) is the only
# state where the moving-baseline heading is trustworthy.
CARR_SOLN_FIXED = CarrSoln.CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES

# ublox heading fields are stored in degrees scaled by 1e-5.
UBX_DEG_SCALE = 1e-5


def normalize_angle(angle: float) -> float:
    """Wrap an angle in radians to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class HeadingToImu(Node):
    def __init__(self):
        super().__init__('gps_heading_imu')

        self.declare_parameter('input_topic', '/rover/ubx_nav_rel_pos_ned')
        self.declare_parameter('output_topic', '/rover/imu')
        self.declare_parameter('frame_id', 'base_link')
        # Offset between the antenna baseline and the vehicle's forward (+X) axis,
        # measured the same way as the heading (degrees, clockwise/NED). Rover
        # antenna at front => baseline already aligned with forward => 0.
        self.declare_parameter('heading_offset_deg', 0.0)
        self.declare_parameter('require_heading_valid', True)
        self.declare_parameter('require_fixed', True)
        self.declare_parameter('require_gnss_fix_ok', True)
        # Covariance floor (rad^2) used when acc_heading is 0/absent. Default ~ (0.5 deg)^2.
        self.declare_parameter('min_yaw_cov', math.radians(0.5) ** 2)
        # Orientation covariance for roll/pitch (GPS provides none) -> effectively unknown.
        self.declare_parameter('unknown_axis_cov', 1e6)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.heading_offset_deg = self.get_parameter('heading_offset_deg').get_parameter_value().double_value
        self.require_heading_valid = self.get_parameter('require_heading_valid').get_parameter_value().bool_value
        self.require_fixed = self.get_parameter('require_fixed').get_parameter_value().bool_value
        self.require_gnss_fix_ok = self.get_parameter('require_gnss_fix_ok').get_parameter_value().bool_value
        self.min_yaw_cov = self.get_parameter('min_yaw_cov').get_parameter_value().double_value
        self.unknown_axis_cov = self.get_parameter('unknown_axis_cov').get_parameter_value().double_value

        # Default (reliable) QoS matches the ublox driver's SystemDefaultsQoS publisher and
        # is what robot_localization expects on the IMU input.
        self.qos_profile = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.sub = self.create_subscription(
            UBXNavRelPosNED, self.input_topic, self.relpos_callback, 10)
        self.pub = self.create_publisher(Imu, self.output_topic, self.qos_profile)

        self._was_publishing = None  # track gating state changes for clean logging

        self.get_logger().info(
            f'gps_heading_imu: {self.input_topic} -> {self.output_topic} '
            f'(frame_id={self.frame_id}, offset={self.heading_offset_deg} deg, '
            f'require_fixed={self.require_fixed})')

    def relpos_callback(self, msg: UBXNavRelPosNED):
        # Gating: only emit a heading when it is actually trustworthy.
        if self.require_heading_valid and not msg.rel_pos_heading_valid:
            self._note_gated('rel_pos_heading_valid is False')
            return
        if self.require_gnss_fix_ok and not msg.gnss_fix_ok:
            self._note_gated('gnss_fix_ok is False')
            return
        if self.require_fixed and msg.carr_soln.status != CARR_SOLN_FIXED:
            self._note_gated(f'carr_soln.status={msg.carr_soln.status} (not fixed)')
            return

        heading_deg = msg.rel_pos_heading * UBX_DEG_SCALE + self.heading_offset_deg
        yaw = normalize_angle(math.pi / 2.0 - math.radians(heading_deg))

        acc_heading_rad = msg.acc_heading * UBX_DEG_SCALE * math.pi / 180.0
        yaw_var = max(acc_heading_rad ** 2, self.min_yaw_cov)

        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = self.frame_id

        # Yaw-only orientation: roll = pitch = 0.
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = math.sin(yaw / 2.0)
        imu.orientation.w = math.cos(yaw / 2.0)

        # Diagonal orientation covariance: roll/pitch unknown, yaw from accHeading.
        imu.orientation_covariance[0] = self.unknown_axis_cov
        imu.orientation_covariance[4] = self.unknown_axis_cov
        imu.orientation_covariance[8] = yaw_var

        # REP convention: -1 in [0] marks angular velocity / linear acceleration as absent.
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(imu)

        if self._was_publishing is not True:
            self._was_publishing = True
            self.get_logger().info(
                f'Publishing heading: {heading_deg % 360.0:.2f} deg (NED) -> '
                f'yaw {math.degrees(yaw):.2f} deg (ENU), accHdg '
                f'{msg.acc_heading * UBX_DEG_SCALE:.2f} deg, carrSoln=fixed')

    def _note_gated(self, reason: str):
        if self._was_publishing is not False:
            self._was_publishing = False
            self.get_logger().warn(f'Not publishing /rover/imu: {reason}')
        else:
            self.get_logger().debug(f'Gated: {reason}', throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = HeadingToImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            # Context may already be shutdown
            pass


if __name__ == '__main__':
    main()
