from eufs_msgs.msg import WaypointArrayStamped, CarState
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
import rclpy
from rclpy.node import Node
from sbg_driver.msg import SbgImuData, SbgEkfNav

import numpy as np

class Control(Node):
    def __init__(self, name):
        super().__init__(name)
        self.speed = 0
        self.speed_target = 0
        self.car_position = [0.0, 0.0]
        self.current_angle = 0.0
        self.error_buffer = [0]
        self.e_T = [0, 0, 0]
        self.period = 0.04      # the time between updates to the path
        
        self.imu_acc_x = 0
        self.imu_acc_y = 0
        self.imu_d_angle = 0
        self.ekf_vel_x = 0
        self.ekf_vel_y = 0

        # Declare ROS parameters
        self.look_ahead = self.declare_parameter("look_ahead", 4.0).value
        self.L = self.declare_parameter("L", 1.5).value
        self.K_p = self.declare_parameter("K_p", 1.0).value
        self.K_i = self.declare_parameter("K_i", 1.0).value
        self.K_d = self.declare_parameter("K_d", 1.0).value
        self.max_lat_acc = self.declare_parameter("max_lat_acc", 5.0).value
        self.safe_speed = self.declare_parameter("safe_speed", 1.5).value
        self.max_speed = self.declare_parameter("max_speed", 4.5).value
        self.buffer_len = self.declare_parameter("buffer_len", 30).value

        # Create subscribers
        self.path_sub = self.create_subscription(WaypointArrayStamped, "/trajectory", self.path_callback, 1)
        self.car_state_sub = self.create_subscription(CarState, "/ground_truth/state", self.state_callback, 1)
        
        self.imu_sub = self.create_subscription(SbgImuData, "/sbg/imu_data", self.imu_callback, 1)
        self.ekf_sub = self.create_subscription(SbgEkfNav, "/sbg/ekf_nav", self.ekf_callback, 1)

        # Create publishers
        self.comand_pub = self.create_publisher(AckermannDriveStamped, "/cmd", 1)
        self.viz_pub = self.create_publisher(Marker, "/control/viz", 1)

    def state_callback(self, msg):
        self.speed = msg.twist.twist.linear.x
        
        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y
        self.car_position = [car_x, car_y]
        
        self.current_angle = msg.pose.pose.orientation.z * 2

    def path_callback(self, msg):
        path = self.convert(msg, "np")  # if you remove the "np" parameter the path will be a 2d array [[x1,y1], ...]
        if len(path) == 0: return

        # Index of the waypoint to the look ahead distance
        look_ahead_index = self.get_look_ahead_index(path)

        # Steering control
        steering_cmd = self.get_steering(path, look_ahead_index)

        # Speed control
        speed_target = self.get_speed_target(path, look_ahead_index)
        self.speed_target = speed_target

        # PID control for acceleration
        acceleration_cmd = self.get_acceleration(speed_target)

        # Publish commands
        self.pubish_command(acceleration_cmd, steering_cmd)
        self.publish_visualisation(acceleration_cmd, steering_cmd)
        
        
    def imu_callback(self, msg):
        self.imu_acc_x = msg.accel.x
        self.imu_acc_y = msg.accel.y
        self.imu_d_angle = msg.gyro.z
        
        
    def ekf_callback(self, msg):
        self.ekf_vel_x = msg.velocity.x
        self.ekf_vel_y = msg.velocity.y
        
        
    def find_distance(self, point1, point2):
        d_x = point1[0] - point2.real
        d_y = point1[1] - point2.imag
        
        return (np.sqrt(d_x**2 + d_y**2))

    def get_look_ahead_index(self, path):
        """
        IMPLEMENT YOURSELF

        :param path: array of complex numbers
        :return: index of waypoint closest to look ahead distance
        """
        
        i = 0
        distance = self.find_distance(self.car_position, path[i])
        
        while distance < 3:
            i = i+1
            distance = self.find_distance(self.car_position, path[i])
            
        return i

    def get_steering(self, path, look_ahead_ind):
        """
        IMPLEMENT YOURSELF
        note: the wheelbase of the car L is saved in the self.L variable
        :param path: array of complex numbers
        :param look_ahead_ind:
        :return: steering angle to be sent to the car
        """
        
        look_ahead_point = path[look_ahead_ind]
        
        distance = self.find_distance(self.car_position, look_ahead_point)
        alpha = self.current_angle + np.arctan(look_ahead_point.imag / look_ahead_point.real)
        steering_angle = np.arctan(2*self.L*np.sin(alpha) / distance)*2.5
        
        '''self.get_logger().info('Point: "%s"' % look_ahead_point)
        self.get_logger().info('Distance: "%s"' % distance)
        self.get_logger().info('Alpha: "%s"' % alpha)
        self.get_logger().info('Steering Angle: "%s"' % steering_angle)'''
        
        return steering_angle

    def get_speed_target(self, path, look_ahead_ind):
        """
        IMPLEMENT YOURSELF
        note: You might want to use the max_lat_acc variable to limit lateral acceleration
        and max_speed to limit the maximum speed
        :param path: array of complex numbers
        :param look_ahead_ind:
        :return: speed we want to reach
        """
        
        if np.absolute(self.speed) > 0:
            k = (self.current_angle/self.period) / self.speed
        else:
            k = 99999
            
        if k != 0:
            R = np.absolute(1/k)
            target_speed = np.minimum(np.sqrt(R*self.max_lat_acc), self.max_speed)
        else:
            target_speed = self.max_speed
            
        return np.maximum(0.5, target_speed)
        

    def get_acceleration(self, speed_target):
        """
        IMPLEMENT YOURSELF
        Note: the current speed of the car is saved in self.speed
        the PID gains are saved in self.K_p, self.K_i, self.K_d
        :param speed_target: speed we want to achieve
        :return: acceleration command to be sent to the car
        """

        '''
        e_t = speed_target - self.speed
        self.e_T.append(e_t)
        
        while len(self.e_T) > 20:
            self.e_T.pop(0)
        
        P = self.K_p * e_t
        I = self.K_i * np.trapz(self.e_T)
        D = self.K_d * (e_t - self.e_T[-2]) / self.period
        
        u_t = P + I + D

        return u_t
        return (2 - np.log(e_t)) / 2'''
        
        u_t = np.sqrt(self.imu_acc_x**2 + self.imu_acc_y**2)
        
        '''self.get_logger().info('\n\n"%s"' % u_t)'''
        
        return u_t
        

    def pubish_command(self, acceleration, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pure_pursuit"
        
        steering = np.arctan(1.54 * self.imu_d_angle / self.ekf_vel_x)
        msg.drive.steering_angle = steering
        msg.drive.acceleration = self.imu_acc_x
        '''msg.drive.speed = self.ekf_vel_x'''
        
        self.get_logger().info("%s" % self.ekf_vel_x)

        self.comand_pub.publish(msg)



    def publish_visualisation(self, acceleration, steering_angle):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_footprint"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = 3.0
        marker.pose.position.y = 4.0
        marker.pose.position.z = 1.0
        marker.id = 0
        marker.ns = "controls"
        marker.id = 0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.5
        marker.text = f" Speed: {round(self.speed, 1)} \n " \
                      f"Target Speed: {round(self.speed_target, 1)} \n " \
                      f"Acceleration: {round(acceleration, 1)} \n " \
                      f"Angle: {round(self.current_angle, 1)} \n " \
                      f"Steering: {round(steering_angle, 1)}"

        self.viz_pub.publish(marker)



    def convert(self, waypoints, struct = ''):
        """
        Converts a cone array message into a np array of complex or 2d list
        :param cones: ConeArrayWithCovariance
        :param struct: Type of output list
        :return:
        """
        if struct == "np":
            return [p.position.x + 1j * p.position.y for p in waypoints.waypoints]
        else:
            return [[p.position.x, p.position.y] for p in waypoints.waypoints]


def main():
    rclpy.init(args=None)
    node = Control("pure_pursuit")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
