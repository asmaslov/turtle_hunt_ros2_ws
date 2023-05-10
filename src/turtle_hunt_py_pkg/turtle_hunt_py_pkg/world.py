#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
import random
import math
from functools import partial

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from std_msgs.msg import String
from turtlesim.msg import Pose
from custom_interfaces.msg import TargetPosition, MultipleTargets

from std_srvs.srv import SetBool, Trigger, Empty
from turtlesim.srv import Spawn, Kill


TARGET_MIN_POSITION = 0.5
TARGET_MAX_POSITION = 10.5
TARGET_MIN_DIAMETER = 0.8
TARGET_MAX_DIAMETER = 1.0
TARGET_POSITION_LOOKUP_ATTEMPTS = 10

PUBLISH_ALL_VISIBLE_TAGETS_PERIOD = 0.05

DEFAULT_AUTO_GENERATE = True
DEFAULT_PERIOD = 3.0
DEFAULT_VISIBLE_RANGE = 3.0


class WorldNode(Node):
    def __init__(self):
        super().__init__('world')

        param_desc_auto_generate = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL,
                                                       description='Enable targets auto generation')
        self.declare_parameter('auto_generate', DEFAULT_AUTO_GENERATE,
                               descriptor=param_desc_auto_generate)
        param_desc_period = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                description='Targets auto generation period')
        self.declare_parameter('period', DEFAULT_PERIOD,
                               descriptor=param_desc_period)
        param_desc_visible_range = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                       description='Visible range for targets')
        self.declare_parameter('visible_range', DEFAULT_VISIBLE_RANGE,
                               descriptor=param_desc_visible_range)

        self.auto_generate = self.get_parameter('auto_generate').value
        self.period = self.get_parameter('period').value
        self.visible_range = self.get_parameter('visible_range').value

        self.timer_add_target = None
        self.timer_publish_all_visible_targets = self.create_timer(
            PUBLISH_ALL_VISIBLE_TAGETS_PERIOD, self.publish_all_visible_targets)

        self.enabled = False
        self.add_target_busy = False

        self.hunter_pos_obtained = False
        self.hunter_x = 0.0
        self.hunter_y = 0.0
        self.hunter_yaw = 0.0

        self.targets = []  # {'x':double, 'y':double, 'd':double, 'name':str}

        # Topics write
        self.hit_publisher = self.create_publisher(
            String, 'hit', QoSPresetProfiles.get_from_short_key('services_default'))
        self.all_visible_targets_publisher = self.create_publisher(
            MultipleTargets, 'all_visible_targets', QoSPresetProfiles.get_from_short_key('services_default'))
        # Topics read
        self.hunter_pose_subscriber = self.create_subscription(
            Pose, 'turtle1/pose', self.callback_hunter_pose, QoSPresetProfiles.get_from_short_key('sensor_data'))
        # Services give
        self.enable_server = self.create_service(
            SetBool, 'enable', self.callback_enable)
        self.new_target_server = self.create_service(
            Trigger, 'new_target', self.callback_new_target)
        self.fire_server = self.create_service(
            Empty, 'fire', self.callback_fire)
        # Services take
        self.target_spawn_client = self.create_client(Spawn, 'spawn')
        while not self.target_spawn_client.wait_for_service(1):
            self.get_logger().warn('Waiting for server turtlesim...')
        self.target_kill_client = self.create_client(Kill, 'kill')
        while not self.target_kill_client.wait_for_service(1):
            self.get_logger().warn('Waiting for server turtlesim...')

        self.get_logger().info('Python version has been started')

    def add_target(self):
        if not self.add_target_busy and self.enabled and self.hunter_pos_obtained:
            self.add_target_busy = True
            good_position = False
            attempts = TARGET_POSITION_LOOKUP_ATTEMPTS
            while attempts > 0:
                attempts -= 1
                random_x = random.uniform(
                    TARGET_MIN_POSITION, TARGET_MAX_POSITION)
                random_y = random.uniform(
                    TARGET_MIN_POSITION, TARGET_MAX_POSITION)
                random_d = random.uniform(
                    TARGET_MIN_DIAMETER, TARGET_MAX_DIAMETER)
                good_position = True
                for target in self.targets:
                    # If the new target is covering any other target
                    if math.sqrt(pow(random_x - target['x'], 2) + pow(random_y - target['y'], 2)) < \
                            random_d / 2 + target['d'] / 2:
                        good_position = False
                # If the new target is inside the visible range
                if math.sqrt(pow(random_x - self.hunter_x, 2) + pow(random_y - self.hunter_y, 2)) < \
                        random_d / 2 + self.visible_range:
                    good_position = False
                if good_position:
                    break
                self.get_logger().warn('Looking for better place for the target...')
            if good_position:
                self.spawn(random_x, random_y, random_d)
                result = True
            else:
                self.get_logger().error('Unable to find the good place for the new target')
                result = False
            self.add_target_busy = False
        else:
            self.get_logger().error('Unable to add target')
            result = False
        return result

    # Topics write
    def publish_hit(self, name):
        msg = String()
        msg.data = name
        self.hit_publisher.publish(msg)

    def publish_all_visible_targets(self):
        if self.hunter_pos_obtained and len(self.targets) > 0:
            msg = MultipleTargets()
            for target in self.targets:
                delta_x = target['x'] - self.hunter_x
                delta_y = target['y'] - self.hunter_y
                # If target is inside the visible range
                if math.sqrt(pow(delta_x, 2) + pow(delta_y, 2)) < target['d'] + self.visible_range:
                    position = TargetPosition()
                    if delta_x == 0 or delta_y == 0:
                        position.rel_angle = 0.0
                    else:
                        # Absolute angle of the target [-pi:pi]
                        target_angle = math.atan(delta_y / abs(delta_x))
                        if delta_x < 0:
                            target_angle = math.copysign(math.pi - abs(target_angle), target_angle)
                        # Relative angle to the target [-pi:pi]
                        position.rel_angle = target_angle - self.hunter_yaw
                        if abs(position.rel_angle) > math.pi:
                            position.rel_angle = math.copysign(
                                2 * math.pi - abs(position.rel_angle), -position.rel_angle)
                    # Distance between centers
                    position.rel_distance = math.sqrt(
                        pow(delta_x, 2) + pow(delta_y, 2))
                    msg.positions.append(position)
            if len(msg.positions) > 0:
                self.all_visible_targets_publisher.publish(msg)

    # Topics read
    def callback_hunter_pose(self, msg: Pose):
        self.hunter_x = msg.x
        self.hunter_y = msg.y
        self.hunter_yaw = msg.theta
        self.hunter_pos_obtained = True

    # Services give
    def callback_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data and not self.enabled and self.auto_generate:
            self.timer_add_target = self.create_timer(
                self.period, self.add_target)
        if not request.data and self.timer_add_target is not None:
            self.destroy_timer(self.timer_add_target)
            self.timer_add_target = None
        self.enabled = request.data
        response.success = True
        response.message = 'Targets disposal enabled' if self.enabled else 'Targets disposal disabled'
        return response

    def callback_new_target(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.enabled:
            if self.add_target():
                response.success = True
                response.message = 'Target disposed'
            else:
                response.success = True
                response.message = 'Unable to dispose the target'
        else:
            response.success = False
            response.message = 'Targets disposal is disabled'
        return response

    def callback_fire(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        target_name = ''
        for target in self.targets:
            # If the hunter center point is covering the target
            if math.sqrt(pow(self.hunter_x - target['x'], 2) + pow(self.hunter_y - target['y'], 2)) < \
                    target['d'] / 2:
                target_name = target['name']
                self.targets.remove(target)
                break
        if target_name:
            self.kill(target_name)
            self.publish_hit(target_name)
        return response

    # Services take
    def spawn(self, x, y, d):
        if self.target_spawn_client.service_is_ready():
            request = Spawn.Request()
            request.x = x
            request.y = y
            future = self.target_spawn_client.call_async(request)
            future.add_done_callback(
                partial(self.callback_spawn, x=x, y=y, d=d))
        else:
            self.get_logger().warn('Server turtlesim not ready')

    def callback_spawn(self, future, x, y, d):
        try:
            response = future.result()
            self.targets.append(
                {'x': x, 'y': y, 'd': d, 'name': response.name})
            self.get_logger().info(
                'Spawned "{}" at ({}, {}) with diameter {}'.format(response.name, x, y, d))
        except Exception as e:
            self.get_logger().error('Server turtlesim call failed %r' % (e, ))

    def kill(self, name):
        if self.target_kill_client.service_is_ready():
            request = Kill.Request()
            request.name = name
            self.target_kill_client.call_async(request)
        else:
            self.get_logger().warn('Server turtlesim not ready')


def main(args=None):
    rclpy.init(args=args)
    node = WorldNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
