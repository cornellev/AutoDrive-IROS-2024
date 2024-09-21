from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
import numpy as np

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # Set up the TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_timestamp = None
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(JointState, '/autodrive/f1tenth_1/left_encoder', self.encoder_left_callback, 10)
        self.create_subscription(JointState, '/autodrive/f1tenth_1/right_encoder', self.encoder_right_callback, 10)
        self.create_subscription(Float64, '/autodrive/f1tenth_1/steering', self.steering_callback, 10)

        self.data = []

    def imu_callback(self, msg):
        self.imu_data = {
            'linear_acceleration': msg.linear_acceleration,
            'angular_velocity': msg.angular_velocity,
            'timestamp': self.get_clock().now().to_msg()
        }

    def encoder_left_callback(self, msg: JointState):
        # Extract position and velocity for the left encoder
        self.encoder_left_data = {
            'position': msg.position[0],  # Assuming the first position entry is the wheel position
            'velocity': msg.velocity[0],  # Assuming the first velocity entry is the wheel velocity
            'timestamp': self.get_clock().now().to_msg()
        }

    def encoder_right_callback(self, msg: JointState):
        # Extract position and velocity for the right encoder
        self.encoder_right_data = {
            'position': msg.position[0],  # Assuming the first position entry is the wheel position
            'velocity': msg.velocity[0],  # Assuming the first velocity entry is the wheel velocity
            'timestamp': self.get_clock().now().to_msg()
        }

    def steering_callback(self, msg):
        self.steering_angle = msg.data

    def lookup_transform(self):
        try:
            # Lookup the transform from 'f1tenth_1' to 'world'
            transform: TransformStamped = self.tf_buffer.lookup_transform('world', 'f1tenth_1', rclpy.time.Time())
            return transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Error getting transform: {e}')
            return None

    def collect_data(self):
        current_timestamp = self.get_clock().now().to_msg()
        if self.last_timestamp is not None:
            time_delta = (current_timestamp.sec - self.last_timestamp.sec) + \
                         (current_timestamp.nanosec - self.last_timestamp.nanosec) * 1e-9
        else:
            time_delta = 0.01  # Default time delta if it's the first reading

        # Get the transform from 'f1tenth_1' to 'world'
        transform = self.lookup_transform()
        if transform:
            position = transform.transform.translation
            orientation = transform.transform.rotation

            # Store the data with position and orientation from the transform
            self.data.append({
                'imu': self.imu_data,
                'encoder_left': self.encoder_left_data,
                'encoder_right': self.encoder_right_data,
                'steering_angle': self.steering_angle,
                'position': {
                    'x': position.x,
                    'y': position.y,
                    'z': position.z
                },
                'orientation': {
                    'x': orientation.x,
                    'y': orientation.y,
                    'z': orientation.z,
                    'w': orientation.w
                },
                'time_delta': time_delta
            })

        self.last_timestamp = current_timestamp
