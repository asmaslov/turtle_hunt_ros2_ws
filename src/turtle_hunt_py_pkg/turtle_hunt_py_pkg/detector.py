#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
import random

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from custom_interfaces.msg import TargetPosition, MultipleTargets


DEFAULT_ERROR = 0.1


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector')

        param_desc_error = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                               description='Maximum error in target values measurement')
        self.declare_parameter('error', DEFAULT_ERROR,
                               descriptor=param_desc_error)

        self.error = self.get_parameter('error').value

        # Topics write
        self.nearest_target_publisher = self.create_publisher(
            TargetPosition, 'nearest_target', QoSPresetProfiles.get_from_short_key('services_default'))
        # Topics read
        self.all_visible_targets_subscriber = self.create_subscription(
            MultipleTargets, 'all_visible_targets', self.callback_all_visible_targets, QoSPresetProfiles.get_from_short_key('services_default'))
        # Services give

        # Services take

        self.get_logger().info('Python version has been started')

    # Topics write
    def publish_nearest_target(self, angle, distance):
        msg = TargetPosition()
        angle_error = random.uniform(-DEFAULT_ERROR / 2, DEFAULT_ERROR / 2)
        msg.rel_angle = angle + angle_error
        distance_error = random.uniform(-DEFAULT_ERROR / 2, DEFAULT_ERROR / 2)
        msg.rel_distance = distance + distance_error
        self.nearest_target_publisher.publish(msg)

    # Topics read
    def callback_all_visible_targets(self, msg: MultipleTargets):
        nearest = min(msg.positions, key=lambda position: position.rel_distance)
        self.publish_nearest_target(nearest.rel_angle, nearest.rel_distance)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
