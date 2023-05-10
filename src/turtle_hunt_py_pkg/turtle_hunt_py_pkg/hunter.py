#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
import math
from scipy import constants

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from geometry_msgs.msg import Vector3, Twist
from turtlesim.msg import Pose
from custom_interfaces.msg import TargetPosition

from std_srvs.srv import SetBool, Empty
from custom_interfaces.srv import AddPoint


POINT_MIN_POSITION = 0.5
POINT_MAX_POSITION = 10.5
CONTROL_PERIOD = 0.01
DISTANCE_MIN = 0.2

K_LINEAR = 2.0
K_ANGULAR = 5.0

DEFAULT_PATROL_SPEED = 2.0
DEFAULT_LINEAR_V_MIN = 1.0
DEFAULT_LINEAR_V_MAX = 5.0
DEFAULT_LINEAR_A_MAX = 3.0
DEFAULT_ROLL_MAX = math.pi / 4


class HunterNode(Node):
    def __init__(self):
        super().__init__('hunter')

        param_desc_patrol_speed = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                      description='Average speed when patrolling from point to point')
        self.declare_parameter('patrol_speed', DEFAULT_PATROL_SPEED,
                               descriptor=param_desc_patrol_speed)
        param_desc_linear_v_min = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                      description='Minimum linear speed')
        self.declare_parameter('linear_v_min', DEFAULT_LINEAR_V_MIN,
                               descriptor=param_desc_linear_v_min)
        param_desc_linear_v_max = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                      description='Maximum linear speed')
        self.declare_parameter('linear_v_max', DEFAULT_LINEAR_V_MAX,
                               descriptor=param_desc_linear_v_max)
        param_desc_linear_a_max = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                      description='Maximum linear acceleration')
        self.declare_parameter('linear_a_max', DEFAULT_LINEAR_A_MAX,
                               descriptor=param_desc_linear_a_max)
        param_desc_roll_max = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                  description='Maximum roll angle (rad)')
        self.declare_parameter('roll_max', DEFAULT_ROLL_MAX,
                               descriptor=param_desc_roll_max)

        self.patrol_speed = self.get_parameter('patrol_speed').value
        self.linear_v_min = self.get_parameter('linear_v_min').value
        self.linear_v_max = self.get_parameter('linear_v_max').value
        self.linear_a_max = self.get_parameter('linear_a_max').value
        self.roll_max = self.get_parameter('roll_max').value

        self.timer_control = None

        self.started = False

        self.tan_roll_max = math.tan(self.roll_max)

        self.pos_obtained = False
        self.x = 0.0
        self.y = 0.0
        # Roll: φ (phi), Pitch: θ (theta), Yaw: ψ (psi)
        self.yaw = 0.0
        self.linear_v = 0.0
        self.angular_v = 0.0

        self.k_linear = K_LINEAR
        self.k_angular = K_ANGULAR

        self.target_distance_min = DISTANCE_MIN
        self.point_distance_min = self.get_rad_min(
            self.patrol_speed) + DISTANCE_MIN

        self.target_locked = False
        self.target_angle = 0.0
        self.target_distance = 0.0

        self.points = []  # {'x':double, 'y':double}
        self.point_idx = 0

        # Topics write
        self.turtle_cmd_publisher = self.create_publisher(
            Twist, 'turtle1/cmd_vel', QoSPresetProfiles.get_from_short_key('services_default'))
        # Topics read
        self.pose_subscriber = self.create_subscription(
            Pose, 'turtle1/pose', self.callback_pose, QoSPresetProfiles.get_from_short_key('sensor_data'))
        self.nearest_target_subscriber = self.create_subscription(
            TargetPosition, 'nearest_target', self.callback_nearest_target, QoSPresetProfiles.get_from_short_key('services_default'))
        # Services give
        self.start_server = self.create_service(
            SetBool, 'start', self.callback_start)
        self.add_mission_point_server = self.create_service(
            AddPoint, 'add_mission_point', self.callback_add_mission_point)
        # Services take
        self.fire_client = self.create_client(Empty, 'fire')
        while not self.fire_client.wait_for_service(1):
            self.get_logger().warn('Waiting for server world...')

        self.get_logger().info('Python version has been started')

    def get_rad_min(self, linear_v):
        return pow(linear_v, 2) / constants.g / self.tan_roll_max

    def get_linear_v(self, linear_v_desired):
        if abs(linear_v_desired - self.linear_v) > self.linear_a_max / CONTROL_PERIOD:
            delta_v = self.linear_a_max / CONTROL_PERIOD
            if linear_v_desired < self.linear_v:
                delta_v *= -1
            linear_v_desired = self.linear_v + delta_v
        if linear_v_desired < self.linear_v_min:
            linear_v_desired = self.linear_v_min
        if linear_v_desired > self.linear_v_max:
            linear_v_desired = self.linear_v_max
        return linear_v_desired

    def get_angular_v(self, angular_v_desired):
        if self.linear_v < self.linear_v_min:
            angular_v_desired = 0.0
        else:
            if angular_v_desired > constants.g / self.linear_v:
                angular_v_desired = constants.g / self.linear_v
        return angular_v_desired

    def control(self):
        if self.pos_obtained:
            if self.target_locked:
                linear_v = self.get_linear_v(
                    self.target_distance * self.k_linear)
                angular_v = self.get_angular_v(
                    self.target_angle * self.k_angular)

                self.publish_turtle_cmd(linear_v, angular_v)

                if self.target_distance < self.target_distance_min:
                    self.fire()
                    self.target_locked = False
            else:
                if len(self.points) > 0:
                    linear_v = self.get_linear_v(self.patrol_speed)
                    delta_x = self.points[self.point_idx]['x'] - self.x
                    delta_y = self.points[self.point_idx]['y'] - self.y
                    # Absolute angle of the point [-pi:pi]
                    point_angle = math.atan(delta_y / abs(delta_x))
                    if delta_x < 0:
                        point_angle = math.copysign(math.pi - abs(point_angle), point_angle)
                    # Relative angle to the target [-pi:pi]
                    rel_angle = point_angle - self.yaw
                    if abs(rel_angle) > math.pi:
                        rel_angle = math.copysign(
                            2 * math.pi - abs(rel_angle), -rel_angle)
                    angular_v = self.get_angular_v(rel_angle * self.k_angular)

                    self.publish_turtle_cmd(linear_v, angular_v)

                    distance = math.sqrt(pow(delta_x, 2) +
                                         pow(delta_y, 2))
                    if distance < self.point_distance_min:
                        self.point_idx += 1
                        if len(self.points) == self.point_idx:
                            self.point_idx = 0

    # Topics write
    def publish_turtle_cmd(self, linear_v, angular_v):
        msg = Twist()
        msg.linear.x = linear_v
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_v
        self.turtle_cmd_publisher.publish(msg)

    # Topics read
    def callback_pose(self, msg: Pose):
        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.theta
        self.linear_v = msg.linear_velocity
        self.angular_v = msg.angular_velocity
        self.pos_obtained = True

    def callback_nearest_target(self, msg: TargetPosition):
        self.target_angle = msg.rel_angle
        self.target_distance = msg.rel_distance
        self.target_locked = True

    # Services give
    def callback_start(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data and not self.started:
            self.timer_control = self.create_timer(
                CONTROL_PERIOD, self.control)
        if not request.data and self.timer_control is not None:
            self.destroy_timer(self.timer_control)
            self.timer_control = None
        self.started = request.data
        response.success = True
        response.message = 'Hunter started' if self.started else 'Hunter stopped'
        return response

    def callback_add_mission_point(self, request: AddPoint.Request, response: AddPoint.Response) -> AddPoint.Response:
        if request.point.x < POINT_MIN_POSITION or request.point.x > POINT_MAX_POSITION or \
                request.point.y < POINT_MIN_POSITION or request.point.y > POINT_MAX_POSITION:
            response.success = False
        else:
            self.points.append({'x': request.point.x, 'y': request.point.y})
            response.success = True
        return response

    # Services take
    def fire(self):
        if self.fire_client.service_is_ready():
            request = Empty.Request()
            self.fire_client.call_async(request)
        else:
            self.get_logger().warn('Server world not ready')


def main(args=None):
    rclpy.init(args=args)
    node = HunterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
