#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from functools import partial

from std_msgs.msg import String

from std_srvs.srv import SetBool, Trigger
from custom_interfaces.srv import AddPoint


POINT_MIN_POSITION = 0.5
POINT_MAX_POSITION = 10.5
TIMER_TICK = 1.0
ENABLE_DELAY = 5.0


class OperatorNode(Node):
    def __init__(self):
        super().__init__('operator')

        # Topics write

        # Topics read
        self.hit_subscriber = self.create_subscription(
            String, 'hit', self.callback_hit, QoSPresetProfiles.get_from_short_key('services_default'))
        # Services give

        # Services take
        self.enable_client = self.create_client(SetBool, 'enable')
        while not self.enable_client.wait_for_service(1):
            self.get_logger().warn('Waiting for server world...')
        self.start_client = self.create_client(SetBool, 'start')
        while not self.start_client.wait_for_service(1):
            self.get_logger().warn('Waiting for server hunter...')
        self.add_mission_point_client = self.create_client(AddPoint, 'add_mission_point')
        while not self.add_mission_point_client.wait_for_service(1):
            self.get_logger().warn('Waiting for server hunter...')
        self.new_target_client = self.create_client(Trigger, 'new_target')
        while not self.new_target_client.wait_for_service(1):
            self.get_logger().warn('Waiting for server world...')

        self.get_logger().info('Python version has been started')

        self.timeout_enable = ENABLE_DELAY

        self.add_mission_point(2.5, 2.5)
        self.add_mission_point(8.5, 2.5)
        self.add_mission_point(8.5, 8.5)
        self.add_mission_point(2.5, 8.5)
        self.start()
        self.get_logger().info('{:.0f}...'.format(self.timeout_enable))
        self.timer_delayed_enable = self.create_timer(TIMER_TICK, self.run_game)

    def run_game(self):
        self.timeout_enable -= TIMER_TICK
        if self.timeout_enable > 0:
            self.get_logger().info('{:.0f}...'.format(self.timeout_enable))
        else:
            self.get_logger().info('Game begins')
            self.destroy_timer(self.timer_delayed_enable)
            self.enable()
            self.new_target()
            self.new_target()
            self.new_target()

    # Topics read
    def callback_hit(self, msg: String):
        self.get_logger().info('Hit {}'.format(msg.data))

    # Services take
    def enable(self):
        if self.enable_client.service_is_ready():
            request = SetBool.Request()
            request.data = True
            future = self.enable_client.call_async(request)
            future.add_done_callback(self.callback_enable)
        else:
            self.get_logger().warn('Server world not ready')

    def callback_enable(self, future):
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error('Server world call failed %r' % (e, ))

    def start(self):
        if self.start_client.service_is_ready():
            request = SetBool.Request()
            request.data = True
            future = self.start_client.call_async(request)
            future.add_done_callback(self.callback_start)
        else:
            self.get_logger().warn('Server hunter not ready')

    def callback_start(self, future):
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error('Server hunter call failed %r' % (e, ))

    def add_mission_point(self, x, y):
        if self.add_mission_point_client.service_is_ready():
            request = AddPoint.Request()
            request.point.x = x
            request.point.y = y
            request.point.z = 0.0
            future = self.add_mission_point_client.call_async(request)
            future.add_done_callback(partial(self.callback_add_mission_point, x=x, y=y))
        else:
            self.get_logger().warn('Server hunter not ready')

    def callback_add_mission_point(self, future, x, y):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Mission point added')
            else:
                self.get_logger().error('Mission point [{}, {}] out of reach'.format(x, y))
        except Exception as e:
            self.get_logger().error('Server hunter call failed %r' % (e, ))

    def new_target(self):
        if self.new_target_client.service_is_ready():
            request = Trigger.Request()
            future = self.new_target_client.call_async(request)
            future.add_done_callback(self.callback_new_target)
        else:
            self.get_logger().warn('Server world not ready')

    def callback_new_target(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('New target disposed')
            else:
                self.get_logger().error('Target not disposed')
        except Exception as e:
            self.get_logger().error('Server world call failed %r' % (e, ))


def main(args=None):
    rclpy.init(args=args)
    node = OperatorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
